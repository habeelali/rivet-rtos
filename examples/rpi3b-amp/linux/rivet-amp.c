// Linux-side tool for running rivet on a core Linux was told not to use.
//
//   rivet-amp probe                 report whether this machine is set up
//   rivet-amp load <image>          copy the image in and release the core
//   rivet-amp console               drain rivet's text console ring
//   rivet-amp trace <file>          drain the PulseTrace ring to a file
//   rivet-amp send <command>        send a command and ring the doorbell
//   rivet-amp bench [rounds]        time the Linux-to-rivet round trip
//
// Build:  cc -O2 -o rivet-amp rivet-amp.c
// Run as root.
//
// Everything here goes through /dev/mem. That works for the reserved
// region only because `mem=` keeps it out of Linux's memory map: with
// CONFIG_STRICT_DEVMEM set, /dev/mem allows mappings of addresses the
// kernel does not consider System RAM, and refuses the rest. The spin
// table at the bottom of memory is the awkward case, and `probe` exists
// to find out which way it goes on a given kernel before anything is
// written.

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <setjmp.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

// Must match RIVET_RPI3B_LOAD_ADDR in the amp package's cargo config.
#define RIVET_BASE   0x30000000UL
#define RIVET_LEN    0x01000000UL   // 16 MiB window the image is linked into

// Must match rivet_bsp_rpi3b::shmem.
#define SHMEM_BASE   0x31000000UL
#define SHMEM_LEN    0x00200000UL
// Two rings in the window: text for a human, and PulseTrace's framed
// binary. They cannot share a transport, because a log line landing in
// the middle of a frame corrupts it.
#define CONSOLE_OFF  0x00000000UL
#define TRACE_OFF    0x00100000UL
// The reverse direction: produced here, consumed by rivet.
#define CMD_OFF      0x00180000UL

// Per-core mailboxes in the ARM-local block. Writing one raises an
// interrupt on the target core, which is the only interrupt on this SoC
// that can be aimed at a specific core; peripheral IRQs go wherever the
// single global routing register points.
#define ARM_LOCAL    0x40000000UL
#define MBOX_SET     0x80UL
#define RING_MAGIC   0x52565443U    // "RVTC"
#define OFF_MAGIC    0
#define OFF_CAP      8
#define OFF_WRITE    16
#define OFF_READ     24
#define OFF_DATA     32

// The firmware's spin table: one 64-bit slot per core.
#define SPIN_TABLE   0xd8UL
#define RIVET_CORE   3

// Everything below goes through an O_SYNC /dev/mem mapping, which on
// AArch64 is Device-nGnRnE memory. That only permits naturally aligned
// accesses of a single register width: memcpy, memset and fread all use
// unaligned and vector stores and will take an alignment fault, which
// surfaces as SIGBUS. So the copies here are explicit aligned stores, and
// the fault is caught rather than killing the process with no clue where.

// Ordering barrier. Guarded so this file still compiles on the host for
// syntax checking; it is only ever *run* on the Pi.
#ifdef __aarch64__
#define BARRIER() __asm__ __volatile__("dsb sy" ::: "memory")
#else
#define BARRIER() __asm__ __volatile__("" ::: "memory")
#endif

static sigjmp_buf fault_env;
static volatile int fault_armed;

static void on_fault(int sig) {
    (void)sig;
    if (fault_armed) siglongjmp(fault_env, 1);
    _exit(135);
}

static void install_fault_handler(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_fault;
    sigaction(SIGBUS, &sa, NULL);
    sigaction(SIGSEGV, &sa, NULL);
}

// Copy with aligned 64-bit stores where possible, bytes for the tail.
// Byte accesses are always naturally aligned, so they are safe here too.
static void copy_to_device(volatile unsigned char *dst, const unsigned char *src, size_t n) {
    size_t i = 0;
    if (((uintptr_t)dst % 8) == 0) {
        volatile uint64_t *d64 = (volatile uint64_t *)dst;
        for (; i + 8 <= n; i += 8) {
            uint64_t v;
            memcpy(&v, src + i, 8);      // source is ordinary memory
            d64[i / 8] = v;
        }
    }
    for (; i < n; i++) dst[i] = src[i];
}

static void zero_device(volatile unsigned char *dst, size_t n) {
    size_t i = 0;
    if (((uintptr_t)dst % 8) == 0) {
        volatile uint64_t *d64 = (volatile uint64_t *)dst;
        for (; i + 8 <= n; i += 8) d64[i / 8] = 0;
    }
    for (; i < n; i++) dst[i] = 0;
}

// mmap only reserves address space; it says nothing about whether the
// physical page behind it can actually be touched. Prove it by writing a
// word and reading it back.
static const char *touch_test(volatile unsigned char *p) {
    if (sigsetjmp(fault_env, 1)) { fault_armed = 0; return "FAULTED on access"; }
    fault_armed = 1;
    volatile uint64_t *w = (volatile uint64_t *)p;
    uint64_t save = *w;
    *w = 0x5aa5f00d0ff0a55aULL;
    uint64_t back = *w;
    *w = save;
    fault_armed = 0;
    return back == 0x5aa5f00d0ff0a55aULL ? "OK" : "reads back wrong";
}

static void *map_phys(int fd, unsigned long base, unsigned long len, int write) {
    long ps = sysconf(_SC_PAGESIZE);
    unsigned long aligned = base & ~(unsigned long)(ps - 1);
    unsigned long slack = base - aligned;
    void *p = mmap(NULL, len + slack, write ? (PROT_READ | PROT_WRITE) : PROT_READ,
                   MAP_SHARED, fd, (off_t)aligned);
    if (p == MAP_FAILED) return NULL;
    return (char *)p + slack;
}

static int cmd_probe(void) {
    int rc = 0;
    printf("== rivet AMP probe ==\n");

    long online = sysconf(_SC_NPROCESSORS_ONLN);
    printf("cpus online              : %ld %s\n", online,
           online == 3 ? "(expected with maxcpus=3)" : "(expected 3)");
    if (online != 3) rc = 1;

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        printf("/dev/mem                 : CANNOT OPEN (%s)\n", strerror(errno));
        return 1;
    }
    printf("/dev/mem                 : open\n");

    struct { const char *name; unsigned long base; } regions[] = {
        { "reserved  0x30000000", RIVET_BASE },
        { "shared    0x31000000", SHMEM_BASE },
        { "spintable 0x000000d8", SPIN_TABLE },
    };
    for (unsigned i = 0; i < 3; i++) {
        void *p = map_phys(fd, regions[i].base, 0x1000, 1);
        if (!p) {
            printf("%s : mmap FAILED (%s)\n", regions[i].name, strerror(errno));
            rc = 1;
            continue;
        }
        const char *res = touch_test((volatile unsigned char *)p);
        printf("%s : mmap OK, write %s\n", regions[i].name, res);
        if (strcmp(res, "OK") != 0) rc = 1;
        munmap(p, 0x1000);
    }

    close(fd);
    printf("verdict                  : %s\n",
           rc == 0 ? "ready" : "not ready, see above");
    return rc;
}

static int cmd_load(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror("open image"); return 1; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || (unsigned long)sz > RIVET_LEN) {
        fprintf(stderr, "image size %ld does not fit the %lu byte window\n", sz, RIVET_LEN);
        fclose(f);
        return 1;
    }

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("/dev/mem"); fclose(f); return 1; }

    // Read into ordinary memory first. fread straight into the device
    // mapping would memcpy with unaligned stores and take a bus error.
    unsigned char *staging = malloc((size_t)sz);
    if (!staging) { fprintf(stderr, "out of memory\n"); fclose(f); close(fd); return 1; }
    if (fread(staging, 1, (size_t)sz, f) != (size_t)sz) {
        fprintf(stderr, "short read of image\n");
        free(staging); fclose(f); close(fd); return 1;
    }
    fclose(f);

    unsigned long span = ((unsigned long)sz + 0xFFFF) & ~0xFFFFUL;
    unsigned char *dst = map_phys(fd, RIVET_BASE, span, 1);
    if (!dst) { perror("mmap reserved region"); free(staging); close(fd); return 1; }

    if (sigsetjmp(fault_env, 1)) {
        fault_armed = 0;
        fprintf(stderr, "bus error while writing %#lx: the mapping is not "
                        "backed by usable memory\n", RIVET_BASE);
        free(staging); close(fd); return 1;
    }
    fault_armed = 1;
    copy_to_device(dst, staging, (size_t)sz);
    fault_armed = 0;
    free(staging);
    printf("loaded %ld bytes at %#lx\n", sz, RIVET_BASE);

    // Read one word back, so a silently-dropped write is not mistaken for
    // a successful load.
    if (*(volatile uint32_t *)dst == 0)
        fprintf(stderr, "warning: first word reads back zero\n");

    // Clear the ring header so `console` does not replay a previous run's
    // output as if it were new.
    // Clear both ring headers, so a drain does not replay a previous
    // run's output as if it were new.
    for (unsigned long off = 0; off <= TRACE_OFF; off += TRACE_OFF) {
        unsigned char *ring = map_phys(fd, SHMEM_BASE + off, 0x1000, 1);
        if (ring) {
            zero_device(ring, 64);
            msync(ring, 64, MS_SYNC);
        }
        if (TRACE_OFF == 0) break;
    }

    // Release the core. The mailbox write has to reach memory rather than
    // sitting in this CPU's cache, because the target core is still
    // running with its caches off; the mapping is uncached, and the
    // barrier below orders it before the wake.
    unsigned char *low = map_phys(fd, 0, 0x1000, 1);
    if (!low) {
        fprintf(stderr,
                "cannot map the spin table at %#lx: %s\n"
                "This kernel's STRICT_DEVMEM refuses low memory. Use the\n"
                "kernel-module fallback described in the README.\n",
                SPIN_TABLE, strerror(errno));
        close(fd);
        return 1;
    }
    volatile uint64_t *slot = (volatile uint64_t *)(low + SPIN_TABLE + RIVET_CORE * 8);
    *slot = (uint64_t)RIVET_BASE;
    msync(low, 0x1000, MS_SYNC);
#ifdef __aarch64__
    // Order the mailbox write ahead of the wake, then wake. Both are
    // unprivileged instructions, so this needs no kernel help.
    __asm__ __volatile__("dsb sy\nsev" ::: "memory");
#else
    fprintf(stderr, "warning: not AArch64, skipping DSB/SEV\n");
#endif
    printf("wrote %#lx to spin slot for core %d, sent SEV\n", RIVET_BASE, RIVET_CORE);

    close(fd);
    return 0;
}

// Drain one ring. `out` NULL means stdout as text; otherwise the bytes
// are written to that file verbatim, which is what the binary trace
// stream needs.
static int drain(unsigned long off, FILE *out) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("/dev/mem"); return 1; }
    unsigned char *ring = map_phys(fd, SHMEM_BASE + off, SHMEM_LEN - off, 1);
    if (!ring) { perror("mmap shared window"); close(fd); return 1; }

    volatile uint32_t *magic = (volatile uint32_t *)(ring + OFF_MAGIC);
    volatile uint32_t *cap   = (volatile uint32_t *)(ring + OFF_CAP);
    volatile uint64_t *wp    = (volatile uint64_t *)(ring + OFF_WRITE);
    volatile uint64_t *rp    = (volatile uint64_t *)(ring + OFF_READ);

    fprintf(stderr, "waiting for the ring at %#lx...\n", SHMEM_BASE + off);
    while (*magic != RING_MAGIC) usleep(20000);
    uint32_t capacity = *cap;
    fprintf(stderr, "ring live, capacity %u bytes. Ctrl-C to stop.\n", capacity);

    unsigned long long total = 0;
    for (;;) {
        uint64_t w = *wp, r = *rp;
        if (w == r) { usleep(10000); continue; }
        // The producer overwrites rather than stalling, so a reader that
        // fell more than a bufferful behind has genuinely lost bytes.
        if (w - r > capacity) {
            fprintf(stderr, "\n[dropped %llu bytes]\n",
                    (unsigned long long)(w - r - capacity));
            r = w - capacity;
        }
        while (r < w) {
            int c = ring[OFF_DATA + (r % capacity)];
            if (out) fputc(c, out); else putchar(c);
            r++;
            total++;
        }
        if (out) {
            fflush(out);
            fprintf(stderr, "\r%llu bytes", total);
        } else {
            fflush(stdout);
        }
        *rp = r;
    }
}

static int cmd_console(void) {
    return drain(CONSOLE_OFF, NULL);
}

static int cmd_trace(const char *path) {
    FILE *out = fopen(path, "wb");
    if (!out) { perror("open trace output"); return 1; }
    fprintf(stderr, "writing PulseTrace frames to %s, Ctrl-C to stop\n", path);
    return drain(TRACE_OFF, out);
}

// Append a command and ring rivet's doorbell.
//
// The write has to be visible before the interrupt, or rivet wakes to an
// empty ring: hence the barrier between them. Both mappings are uncached,
// so no cache maintenance is needed on top of that.
static int cmd_send(const char *text) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("/dev/mem"); return 1; }

    unsigned char *ring = map_phys(fd, SHMEM_BASE + CMD_OFF, 0x10000, 1);
    if (!ring) { perror("mmap command ring"); close(fd); return 1; }
    volatile uint32_t *magic = (volatile uint32_t *)(ring + OFF_MAGIC);
    volatile uint32_t *cap   = (volatile uint32_t *)(ring + OFF_CAP);
    volatile uint64_t *wp    = (volatile uint64_t *)(ring + OFF_WRITE);
    if (*magic != RING_MAGIC) {
        fprintf(stderr, "command ring not initialised: is rivet running?\n");
        close(fd);
        return 1;
    }
    uint32_t capacity = *cap;

    uint64_t w = *wp;
    for (const char *p = text; *p; p++) ring[OFF_DATA + (w++ % capacity)] = (unsigned char)*p;
    ring[OFF_DATA + (w++ % capacity)] = '\n';   // one command per line
    BARRIER();
    *wp = w;
    BARRIER();

    // Ring the doorbell for core 3, mailbox 0.
    unsigned char *local = map_phys(fd, ARM_LOCAL, 0x1000, 1);
    if (!local) { perror("mmap ARM-local block"); close(fd); return 1; }
    volatile uint32_t *set = (volatile uint32_t *)(local + MBOX_SET + RIVET_CORE * 16);
    *set = 1;
    BARRIER();

    printf("sent \"%s\" and rang core %d\n", text, RIVET_CORE);
    close(fd);
    return 0;
}

// Read the architected counter. rivet reads CNTPCT_EL0 and this reads
// CNTVCT_EL0, but CNTVOFF_EL2 is zero on this board, so the two are the
// same number from the same 19.2 MHz counter. That is what makes a true
// one-way latency measurable at all: without a shared timebase, the only
// honest figure is a round trip.
//
// CNTPCT_EL0 itself traps to EL1 from userspace here and dies with SIGILL,
// which is why this is not simply the same register.
static inline uint64_t cntvct(void) {
    uint64_t v;
    __asm__ volatile("isb; mrs %0, cntvct_el0" : "=r"(v) :: "memory");
    return v;
}

#define CNT_HZ 19200000ULL
static uint64_t cnt_to_ns(uint64_t t) { return t * 625ULL / 12ULL; }

struct rt_stats { uint64_t min, max, sum, n; };
static void rt_add(struct rt_stats *s, uint64_t v) {
    if (!s->n || v < s->min) s->min = v;
    if (v > s->max) s->max = v;
    s->sum += v;
    s->n++;
}

// Round trip: Linux writes a timestamped command, rings the doorbell,
// and waits for rivet's reply to appear in the console ring.
//
// The reply is detected by watching the console write pointer move rather
// than by parsing text. Parsing would time the parser as much as the path
// being measured, and the first byte becoming visible is the moment that
// actually matters.
static int cmd_bench(int rounds) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("/dev/mem"); return 1; }

    unsigned char *cmd = map_phys(fd, SHMEM_BASE + CMD_OFF, 0x10000, 1);
    unsigned char *con = map_phys(fd, SHMEM_BASE + CONSOLE_OFF, 0x100000, 1);
    unsigned char *local = map_phys(fd, ARM_LOCAL, 0x1000, 1);
    if (!cmd || !con || !local) { perror("mmap"); close(fd); return 1; }

    volatile uint32_t *cmagic = (volatile uint32_t *)(cmd + OFF_MAGIC);
    volatile uint32_t *ccap   = (volatile uint32_t *)(cmd + OFF_CAP);
    volatile uint64_t *cwp    = (volatile uint64_t *)(cmd + OFF_WRITE);
    volatile uint32_t *ocap   = (volatile uint32_t *)(con + OFF_CAP);
    volatile uint64_t *owp    = (volatile uint64_t *)(con + OFF_WRITE);
    volatile uint64_t *orp    = (volatile uint64_t *)(con + OFF_READ);
    volatile uint32_t *bell   = (volatile uint32_t *)(local + MBOX_SET + RIVET_CORE * 16);

    if (*cmagic != RING_MAGIC) {
        fprintf(stderr, "command ring not initialised: is rivet running?\n");
        close(fd);
        return 1;
    }
    uint32_t ccapacity = *ccap, ocapacity = *ocap;

    struct rt_stats rt = {0};
    uint64_t timeouts = 0;

    for (int i = 0; i < rounds; i++) {
        // Discard anything already queued, so the pointer only moves for
        // the reply this round is about to provoke.
        *orp = *owp;
        BARRIER();

        char line[64];
        uint64_t t0 = cntvct();
        int len = snprintf(line, sizeof line, "ts %llu\n", (unsigned long long)t0);

        uint64_t w = *cwp;
        for (int k = 0; k < len; k++) cmd[OFF_DATA + (w++ % ccapacity)] = (unsigned char)line[k];
        BARRIER();
        *cwp = w;
        BARRIER();
        *bell = 1;
        BARRIER();

        // Spin rather than sleep: the whole quantity being measured is
        // smaller than the shortest sleep this process could ask for.
        uint64_t start = *orp, deadline = t0 + CNT_HZ / 10;   // 100 ms
        for (;;) {
            if (*owp != start) { rt_add(&rt, cntvct() - t0); break; }
            if (cntvct() > deadline) { timeouts++; break; }
        }
        usleep(1000);   // let rivet settle between rounds
    }

    printf("== Linux to rivet round trip ==\n");
    if (rt.n) {
        printf("  command ring -> doorbell -> reply visible\n");
        printf("  min %llu  mean %llu  max %llu ns   n=%llu\n",
               (unsigned long long)cnt_to_ns(rt.min),
               (unsigned long long)cnt_to_ns(rt.sum / rt.n),
               (unsigned long long)cnt_to_ns(rt.max),
               (unsigned long long)rt.n);
    } else {
        printf("  no replies: is a build with a command handler loaded?\n");
    }
    if (timeouts) printf("  timeouts: %llu\n", (unsigned long long)timeouts);

    // One-way throughput in the rivet-to-Linux direction. rivet floods the
    // console ring and this drains it as fast as it can. The producer
    // overwrites rather than blocking, so a shortfall against the amount
    // asked for is a real measurement of the consumer, not an error.
    const unsigned kib = 256;
    *orp = *owp;
    BARRIER();
    char flood[32];
    int len = snprintf(flood, sizeof flood, "flood %u\n", kib);
    uint64_t w = *cwp;
    for (int k = 0; k < len; k++) cmd[OFF_DATA + (w++ % ccapacity)] = (unsigned char)flood[k];
    BARRIER();
    *cwp = w;
    BARRIER();
    *bell = 1;
    BARRIER();

    uint64_t t0 = cntvct(), last = t0, got = 0, dropped = 0;
    for (;;) {
        uint64_t ww = *owp, rr = *orp;
        if (ww == rr) {
            // Two idle seconds after the last byte means it is over.
            if (cntvct() - last > 2 * CNT_HZ) break;
            continue;
        }
        if (ww - rr > ocapacity) { dropped += ww - rr - ocapacity; rr = ww - ocapacity; }
        // Read 64 bits at a time where the window allows it. The shared
        // mapping is Device memory, where each access goes to the
        // interconnect on its own: a byte-at-a-time loop measures the
        // cost of that round trip roughly eight times over and reports it
        // as the ring's throughput, which it is not. Unaligned and
        // vector accesses fault on Device memory, so the fast path is
        // taken only when the offset is aligned and the run does not
        // wrap.
        volatile uint64_t sink = 0;
        while (rr < ww) {
            size_t off = rr % ocapacity;
            uint64_t run = ww - rr;
            if (run > ocapacity - off) run = ocapacity - off;
            if ((off % 8) == 0 && run >= 8) {
                uint64_t words = run / 8;
                const volatile uint64_t *p = (const volatile uint64_t *)(con + OFF_DATA + off);
                for (uint64_t k = 0; k < words; k++) sink ^= p[k];
                rr += words * 8;
                got += words * 8;
            } else {
                sink ^= con[OFF_DATA + off];
                rr++;
                got++;
            }
        }
        (void)sink;
        *orp = rr;
        last = cntvct();
    }
    uint64_t elapsed = last - t0;
    printf("\n== ring one-way, rivet to Linux ==\n");
    if (got && elapsed) {
        printf("  %llu KiB drained in %llu us = %llu MiB/s\n",
               (unsigned long long)(got / 1024),
               (unsigned long long)(cnt_to_ns(elapsed) / 1000),
               (unsigned long long)((got * CNT_HZ) / (elapsed * 1024 * 1024)));
    } else {
        printf("  nothing arrived: the loaded build has no flood command\n");
    }
    if (dropped)
        printf("  %llu KiB overwritten before this reader took them\n",
               (unsigned long long)(dropped / 1024));

    close(fd);
    return 0;
}

int main(int argc, char **argv) {
    install_fault_handler();
    if (argc < 2) {
        fprintf(stderr,
                "usage: %s probe | load <image> | console | trace <file>\n"
                "       %s send <cmd> | bench [rounds]\n",
                argv[0],
                argv[0]);
        return 2;
    }
    if (!strcmp(argv[1], "probe"))   return cmd_probe();
    if (!strcmp(argv[1], "bench"))   return cmd_bench(argc > 2 ? atoi(argv[2]) : 500);
    if (!strcmp(argv[1], "console")) return cmd_console();
    if (!strcmp(argv[1], "send")) {
        if (argc < 3) { fprintf(stderr, "send needs a command\n"); return 2; }
        return cmd_send(argv[2]);
    }
    if (!strcmp(argv[1], "trace")) {
        if (argc < 3) { fprintf(stderr, "trace needs an output path\n"); return 2; }
        return cmd_trace(argv[2]);
    }
    if (!strcmp(argv[1], "load")) {
        if (argc < 3) { fprintf(stderr, "load needs an image path\n"); return 2; }
        return cmd_load(argv[2]);
    }
    fprintf(stderr, "unknown command %s\n", argv[1]);
    return 2;
}
