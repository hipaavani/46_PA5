#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <math.h>
#include <time.h>

static size_t page_size;

#define align_down(x, a) ((x) & ~((typeof(x))(a) - 1))

#define AS_LIMIT  (1 << 23)
#define MAX_SQRTS (1 << 27)

#ifndef CHUNK_PAGES
#define CHUNK_PAGES 1
#endif

#ifndef RESIDENT_CHUNKS
#define RESIDENT_CHUNKS 1
#endif

#if CHUNK_PAGES < 1 || RESIDENT_CHUNKS < 1
#error "CHUNK_PAGES and RESIDENT_CHUNKS must be positive"
#endif

static double *sqrts;

static uintptr_t resident[RESIDENT_CHUNKS];
static int resident_count = 0;
static int fifo_head = 0;

static volatile sig_atomic_t fault_count = 0;
static volatile sig_atomic_t random_faults = 0;
static volatile sig_atomic_t sequential_faults = 0;
static volatile sig_atomic_t access_phase = 0;

static void
calculate_sqrts(double *sqrt_pos, int start, int nr)
{
    for (int i = 0; i < nr; i++)
        sqrt_pos[i] = sqrt((double)(start + i));
}
static void
map_chunk(uintptr_t chunk_addr)
{
    size_t chunk_bytes = CHUNK_PAGES * page_size;

    // Check whether this chunk is already resident.
    for (int i = 0; i < resident_count; i++) {
        int slot = (fifo_head + i) % RESIDENT_CHUNKS;

        if (resident[slot] == chunk_addr)
            return;
    }

    int slot;

    // Evict the oldest chunk if the resident array is full.
    if (resident_count == RESIDENT_CHUNKS) {

        if (munmap((void *)resident[fifo_head], chunk_bytes) == -1) {
            const char msg[] = "munmap failed\n";
            write(STDERR_FILENO, msg, sizeof(msg) - 1);
            _exit(EXIT_FAILURE);
        }

        slot = fifo_head;
        fifo_head = (fifo_head + 1) % RESIDENT_CHUNKS;
    }
    else {
        slot = (fifo_head + resident_count) % RESIDENT_CHUNKS;
        resident_count++;
    }

    void *mapped = mmap(
        (void *)chunk_addr,
        chunk_bytes,
        PROT_READ | PROT_WRITE,
        MAP_FIXED | MAP_PRIVATE | MAP_ANONYMOUS,
        -1,
        0
    );

    if (mapped == MAP_FAILED) {
        const char msg[] = "mmap failed\n";
        write(STDERR_FILENO, msg, sizeof(msg) - 1);
        _exit(EXIT_FAILURE);
    }

    resident[slot] = chunk_addr;

    int start = (chunk_addr - (uintptr_t)sqrts) / sizeof(double);
    int nr = chunk_bytes / sizeof(double);

    calculate_sqrts((double *)mapped, start, nr);
}
static void
handle_sigsegv(int sig, siginfo_t *si, void *ctx)
{
    (void)sig;
    (void)ctx;

    fault_count++;

    if (access_phase == 0)
        random_faults++;
    else
        sequential_faults++;

    uintptr_t fault_addr = (uintptr_t)si->si_addr;
    uintptr_t table_start = (uintptr_t)sqrts;
    uintptr_t table_end = table_start + MAX_SQRTS * sizeof(double);

    if (fault_addr < table_start || fault_addr >= table_end) {
        const char msg[] = "Invalid SIGSEGV\n";
        write(STDERR_FILENO, msg, sizeof(msg) - 1);
        _exit(EXIT_FAILURE);
    }

    size_t chunk_bytes = CHUNK_PAGES * page_size;

    uintptr_t chunk_addr =
        table_start +
        ((fault_addr - table_start) / chunk_bytes) * chunk_bytes;

    // Map the chunk that caused the fault.
    map_chunk(chunk_addr);

#if RESIDENT_CHUNKS > 1
    // Prefetch the next chunk if it is within the table.
    uintptr_t next_chunk = chunk_addr + chunk_bytes;

if (next_chunk < table_end) {
    map_chunk(next_chunk);

    if (madvise((void *)next_chunk,
                chunk_bytes, MADV_WILLNEED) == -1) {
        const char msg[] = "madvise WILLNEED failed\n";
        write(STDERR_FILENO, msg, sizeof(msg) - 1);
        _exit(EXIT_FAILURE);
    }
}
#endif
}

static void
setup_sqrt_region(void)
{
    struct rlimit lim = {AS_LIMIT, AS_LIMIT};
    struct sigaction act;

    sqrts = mmap(
        NULL,
        MAX_SQRTS * sizeof(double) + AS_LIMIT,
        PROT_NONE,
        MAP_PRIVATE | MAP_ANONYMOUS,
        -1,
        0
    );

    if (sqrts == MAP_FAILED) {
        perror("mmap");
        exit(EXIT_FAILURE);
    }

    if (munmap(sqrts,
               MAX_SQRTS * sizeof(double) + AS_LIMIT) == -1) {
        perror("munmap");
        exit(EXIT_FAILURE);
    }

    if (setrlimit(RLIMIT_AS, &lim) == -1) {
        perror("setrlimit");
        exit(EXIT_FAILURE);
    }

    act.sa_sigaction = handle_sigsegv;
    act.sa_flags = SA_SIGINFO;
    sigemptyset(&act.sa_mask);

    if (sigaction(SIGSEGV, &act, NULL) == -1) {
        perror("sigaction");
        exit(EXIT_FAILURE);
    }
}

static void
test_sqrt_region(void)
{
    int pos = rand() % (MAX_SQRTS - 1);
    double correct_sqrt;

    printf("Validating square root table contents...\n");
    srand(0xDEADBEEF);

    for (int i = 0; i < 500000; i++) {

        if (i % 2 == 0){
            pos = rand() % (MAX_SQRTS - 1);
access_phase=0;
}
        else{
            pos += 1;
access_phase=1;
}
        calculate_sqrts(&correct_sqrt, pos, 1);

        if (sqrts[pos] != correct_sqrt) {
            fprintf(stderr,
                    "Square root is incorrect. Expected %f, got %f.\n",
                    correct_sqrt, sqrts[pos]);
            exit(EXIT_FAILURE);
        }
    }

    printf("All tests passed!\n");
}

int
main(void)
{
    page_size = sysconf(_SC_PAGESIZE);

    printf("Page size: %ld bytes\n", (long)page_size);
    printf("Chunk pages: %d\n", CHUNK_PAGES);
    printf("Resident chunks: %d\n", RESIDENT_CHUNKS);

    setup_sqrt_region();

    struct timespec start, end;

    clock_gettime(CLOCK_MONOTONIC, &start);

    test_sqrt_region();

    clock_gettime(CLOCK_MONOTONIC, &end);

    struct rusage ru;
    getrusage(RUSAGE_SELF, &ru);

    double elapsed =
        (end.tv_sec - start.tv_sec) +
        (end.tv_nsec - start.tv_nsec) / 1e9;

    printf("\n--- Performance Results ---\n");
    printf("SIGSEGV invocations: %d\n", (int)fault_count);
printf("Random faults: %d\n", (int)random_faults);
printf("Sequential faults: %d\n", (int)sequential_faults);
    printf("Minor faults: %ld\n", ru.ru_minflt);
    printf("Major faults: %ld\n", ru.ru_majflt);
    printf("Peak RSS: %ld KiB\n", ru.ru_maxrss);
    printf("Wall time: %.6f seconds\n", elapsed);

    return 0;
}
