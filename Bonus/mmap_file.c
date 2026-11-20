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
#include <fcntl.h>

static size_t page_size;

#define align_down(x, a) ((x) & ~((typeof(x))(a) - 1))

#define AS_LIMIT  (1 << 23)
#define MAX_SQRTS (1 << 23)

#ifndef CHUNK_PAGES
#define CHUNK_PAGES 1
#endif

#ifndef RESIDENT_CHUNKS
#define RESIDENT_CHUNKS 1
#endif

#ifndef USE_ADVICE
#define USE_ADVICE 0
#endif

#ifndef USE_PREFETCH
#define USE_PREFETCH 0
#endif

#if CHUNK_PAGES < 1 || RESIDENT_CHUNKS < 1
#error "CHUNK_PAGES and RESIDENT_CHUNKS must be positive"
#endif

static double *sqrts;
static int backing_fd = -1;

static uintptr_t resident[RESIDENT_CHUNKS];
static int resident_count = 0;
static int fifo_head = 0;

static volatile sig_atomic_t fault_count = 0;

static void
calculate_sqrts(double *sqrt_pos, int start, int nr)
{
    for (int i = 0; i < nr; i++)
        sqrt_pos[i] = sqrt((double)(start + i));
}
static void
create_backing_file(void)
{
    backing_fd = open("sqrts.bin", O_RDONLY);

    if (backing_fd == -1) {
        if (errno != ENOENT) {
            perror("open sqrts.bin");
            exit(EXIT_FAILURE);
        }

        backing_fd = open("sqrts.bin",
                          O_CREAT | O_EXCL | O_WRONLY, 0644);

        if (backing_fd == -1) {
            perror("create sqrts.bin");
            exit(EXIT_FAILURE);
        }

        double buffer[512];

        for (int start = 0; start < MAX_SQRTS; start += 512) {
            calculate_sqrts(buffer, start, 512);

            size_t written = 0;

            while (written < sizeof(buffer)) {
                ssize_t n = write(backing_fd,
                                  (char *)buffer + written,
                                  sizeof(buffer) - written);

                if (n <= 0) {
                    perror("write sqrts.bin");
                    exit(EXIT_FAILURE);
                }

                written += n;
            }
        }

        close(backing_fd);

        backing_fd = open("sqrts.bin", O_RDONLY);

        if (backing_fd == -1) {
            perror("reopen sqrts.bin");
            exit(EXIT_FAILURE);
        }

        printf("Created sqrts.bin (64 MiB)\n");
    } else {
        printf("Using existing sqrts.bin\n");
    }
}

static void
handle_sigsegv(int sig, siginfo_t *si, void *ctx)
{
    (void)sig;
    (void)ctx;

    fault_count++;

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
        table_start + ((fault_addr - table_start) / chunk_bytes)
                    * chunk_bytes;

    int slot;

    // FIFO eviction if all resident slots are occupied.
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
    off_t file_offset = (off_t)(chunk_addr - (uintptr_t)sqrts);

    void *mapped = mmap((void *)chunk_addr,
                        chunk_bytes,
                        PROT_READ,
                        MAP_PRIVATE | MAP_FIXED,
                        backing_fd,
                        file_offset);

if (mapped == MAP_FAILED) {
    const char msg[] = "mmap failed\n";
    write(STDERR_FILENO, msg, sizeof(msg) - 1);
    _exit(EXIT_FAILURE);
}

#if USE_ADVICE == 1

    if (madvise(mapped, chunk_bytes, MADV_SEQUENTIAL) == -1) {
        const char msg[] = "MADV_SEQUENTIAL failed\n";
        write(STDERR_FILENO, msg, sizeof(msg) - 1);
        _exit(EXIT_FAILURE);
    }

#elif USE_ADVICE == 2

    if (madvise(mapped, chunk_bytes, MADV_RANDOM) == -1) {
        const char msg[] = "MADV_RANDOM failed\n";
        write(STDERR_FILENO, msg, sizeof(msg) - 1);
        _exit(EXIT_FAILURE);
    }

#endif

resident[slot] = chunk_addr;

#if USE_PREFETCH

    if (RESIDENT_CHUNKS > 1) {

        uintptr_t next_chunk = chunk_addr + chunk_bytes;

        if (next_chunk < table_end) {

            int next_slot;

            if (resident_count == RESIDENT_CHUNKS) {

                if (munmap((void *)resident[fifo_head],
                           chunk_bytes) == -1) {
                    const char msg[] = "prefetch munmap failed\n";
                    write(STDERR_FILENO, msg, sizeof(msg) - 1);
                    _exit(EXIT_FAILURE);
                }

                next_slot = fifo_head;
                fifo_head = (fifo_head + 1) % RESIDENT_CHUNKS;
            }
            else {
                next_slot =
                    (fifo_head + resident_count) % RESIDENT_CHUNKS;
                resident_count++;
            }

            off_t next_offset =
                (off_t)(next_chunk - table_start);

            void *next_mapped = mmap(
                (void *)next_chunk,
                chunk_bytes,
                PROT_READ,
                MAP_PRIVATE | MAP_FIXED,
                backing_fd,
                next_offset
            );

            if (next_mapped == MAP_FAILED) {
                const char msg[] = "prefetch mmap failed\n";
                write(STDERR_FILENO, msg, sizeof(msg) - 1);
                _exit(EXIT_FAILURE);
            }

            if (madvise(next_mapped, chunk_bytes,
                        MADV_WILLNEED) == -1) {
                const char msg[] = "MADV_WILLNEED failed\n";
                write(STDERR_FILENO, msg, sizeof(msg) - 1);
                _exit(EXIT_FAILURE);
            }

            resident[next_slot] = next_chunk;
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

        if (i % 2 == 0)
            pos = rand() % (MAX_SQRTS - 1);
        else
            pos += 1;

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

    create_backing_file();
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
    printf("Minor faults: %ld\n", ru.ru_minflt);
    printf("Major faults: %ld\n", ru.ru_majflt);
    printf("Peak RSS: %ld KiB\n", ru.ru_maxrss);
    printf("Wall time: %.6f seconds\n", elapsed);

    return 0;
}
