// Linux-side tool for running rivet on a core Linux was told not to use.
//
//   rivet-amp probe                 report whether this machine is set up
//   rivet-amp load <image>          copy the image in and release the core
//   rivet-amp console               drain rivet's text console ring
//   rivet-amp trace <file>          drain the PulseTrace ring to a file
//   rivet-amp send <command>        send a command and ring the doorbell
//   rivet-amp bench [rounds]        time the Linux-to-rivet round trip
//   rivet-amp scope [n] [ms]        pulse a GPIO and ring, for a scope
//   rivet-amp status                what is running, on which core, where
//   rivet-amp banner                one-line identity, also to /dev/kmsg
//   rivet-amp watch                 exit non-zero if the heartbeat stops
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
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

// Defaults, overridden at run time from the device tree. The addresses
// used to be declared independently here, in the board crate, in the
// cargo config and in provision.sh, with nothing checking they agreed: a
// change to one produced silent corruption rather than an error, because
// every ring magic still matched and every pointer still pointed
// somewhere. read_config() below reads the values the provisioner wrote
// into /reserved-memory, so there is one source of truth and these are
// only the fallback for a card provisioned before that existed.
static unsigned long RIVET_BASE = 0x30000000UL;
static unsigned long SHMEM_BASE = 0x31000000UL;
static int RIVET_CORE_N = 3;
#define RIVET_LEN    0x01000000UL   // 16 MiB window the image is linked into
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
// GPIO, for the scope demonstration. Only the level registers are touched
// here: GPSET and GPCLR are write-to-set and write-to-clear, so driving a
// pin from this side cannot disturb the pins rivet drives in the same
// bank, and no lock is needed between the two. GPFSEL, which is a
// read-modify-write register and would need one, is left entirely to
// rivet's scope_demo image.
#define GPIO_BASE    0x3F200000UL
#define GPSET0       0x1C
#define GPCLR0       0x28
#define GPLEV0       0x34
#define SCOPE_PIN    20
#define ISR_PIN      21
#define TASK_PIN     26

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

// ── System header, mirrored from rivet_bsp_rpi3b::sysinfo ────────────
//
// Offsets are the ABI. Change one and bump SYS_ABI on both sides; the
// handshake below exists so a mismatch is a sentence rather than a
// mystery.
#define SYS_OFFSET       0x1FF000UL
#define SYS_MAGIC        0x52565453U   // "RVTS"
#define SYS_ABI          1U

#define SYS_O_MAGIC      0x00
#define SYS_O_ABI        0x04
#define SYS_O_HEARTBEAT  0x08
#define SYS_O_BOOT       0x10
#define SYS_O_TICK_HZ    0x18
#define SYS_O_CORE       0x1C
#define SYS_O_STATE      0x20
#define SYS_O_BEAT_HZ    0x24
#define SYS_O_LOAD_BASE  0x28
#define SYS_O_SHARED     0x30
#define SYS_O_OWNED_LEN  0x38
#define SYS_O_SYSVER     0x40
#define SYS_O_IMAGE      0x60
#define SYS_O_BUILD      0x80
#define SYS_O_RIVETVER   0xB0

struct sysinfo_hdr {
    uint32_t magic, abi;
    uint64_t heartbeat, boot;
    uint32_t tick_hz, core, state, beat_hz;
    uint64_t load_base, shared, owned_len;
    char sysver[33], image[33], build[49], rivetver[33];
};

static const char *state_name(uint32_t s) {
    switch (s) {
    case 0: return "booting";
    case 1: return "running";
    case 2: return "faulted";
    case 3: return "exited";
    default: return "unknown";
    }
}

// ── Colour ───────────────────────────────────────────────────────────
//
// Only when stdout is a terminal. Piping this into a log or a journal
// should not fill it with escape sequences.
static int use_colour = 0;
#define C(code) (use_colour ? code : "")
#define CDIM    C("\033[2m")
#define CBOLD   C("\033[1m")
#define CRED    C("\033[31m")
#define CGRN    C("\033[32m")
#define CYEL    C("\033[33m")
#define CBLU    C("\033[34m")
#define CCYN    C("\033[36m")
#define CRST    C("\033[0m")

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
        { "reserved region", RIVET_BASE },
        { "shared window", SHMEM_BASE },
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
    volatile uint64_t *slot = (volatile uint64_t *)(low + SPIN_TABLE + RIVET_CORE_N * 8);
    *slot = (uint64_t)RIVET_BASE;
    msync(low, 0x1000, MS_SYNC);
#ifdef __aarch64__
    // Order the mailbox write ahead of the wake, then wake. Both are
    // unprivileged instructions, so this needs no kernel help.
    __asm__ __volatile__("dsb sy\nsev" ::: "memory");
#else
    fprintf(stderr, "warning: not AArch64, skipping DSB/SEV\n");
#endif
    printf("wrote %#lx to spin slot for core %d, sent SEV\n", RIVET_BASE, RIVET_CORE_N);

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
    volatile uint32_t *set = (volatile uint32_t *)(local + MBOX_SET + RIVET_CORE_N * 16);
    *set = 1;
    BARRIER();

    printf("sent \"%s\" and rang core %d\n", text, RIVET_CORE_N);
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
    volatile uint32_t *bell   = (volatile uint32_t *)(local + MBOX_SET + RIVET_CORE_N * 16);

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

// Raise a pin, ring the doorbell, and do it again, so an oscilloscope
// triggering on that pin can measure the interval to the pins rivet
// raises.
//
// The GPIO write comes first and the mailbox write second, so the cost of
// the mailbox write itself falls inside the measured interval. That is
// the honest ordering: it is work Linux must do before rivet can possibly
// hear anything.
//
// This deliberately does not write to the command ring. rivet's
// scope_demo does not read one, and leaving it out keeps everything
// between the two edges on the path being measured.
static int cmd_scope(int rounds, int period_ms) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("/dev/mem"); return 1; }

    unsigned char *gpio  = map_phys(fd, GPIO_BASE, 0x1000, 1);
    unsigned char *local = map_phys(fd, ARM_LOCAL, 0x1000, 1);
    if (!gpio || !local) { perror("mmap"); close(fd); return 1; }

    volatile uint32_t *set  = (volatile uint32_t *)(gpio + GPSET0);
    volatile uint32_t *clr  = (volatile uint32_t *)(gpio + GPCLR0);
    volatile uint32_t *bell = (volatile uint32_t *)(local + MBOX_SET + RIVET_CORE_N * 16);
    const uint32_t bit = 1u << SCOPE_PIN;

    printf("pulsing GPIO%d (header 38) and ringing core %d, %d times every %d ms\n",
           SCOPE_PIN, RIVET_CORE_N, rounds, period_ms);
    printf("trigger the scope on GPIO%d rising; ground is header 39\n", SCOPE_PIN);

    // Ask for the best scheduling this process can get. It does not make
    // the send deterministic, and the tail of the measured distribution
    // is still Linux's, but it removes the easiest source of outliers.
    struct sched_param sp = { .sched_priority = 50 };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        fprintf(stderr, "note: no SCHED_FIFO (%s), expect a longer tail\n",
                strerror(errno));
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        fprintf(stderr, "note: no mlockall (%s), a page fault may show up\n",
                strerror(errno));

    // Watch the pads while pulsing, so this reports something useful with
    // no instrument attached. A wiring mistake and a broken doorbell path
    // look identical on an unconnected scope, and this separates them:
    // the levels are read back from GPLEV, which reports what the pad is
    // actually doing rather than what was last written to it.
    volatile uint32_t *lev = (volatile uint32_t *)(gpio + GPLEV0);
    int saw_isr = 0, saw_task = 0;

    for (int i = 0; i < rounds; i++) {
        *set = bit;
        BARRIER();
        *bell = 1;
        BARRIER();

        // Poll across the pulse. rivet holds its pins high for 200 us, so
        // a spin here catches them; a sleeping loop would step over them.
        uint64_t until = cntvct() + CNT_HZ / 2000;   // 500 us
        do {
            uint32_t l = *lev;
            if (l & (1u << ISR_PIN))  saw_isr = 1;
            if (l & (1u << TASK_PIN)) saw_task = 1;
        } while (cntvct() < until);

        *clr = bit;
        BARRIER();
        usleep(period_ms * 1000);
    }
    *clr = bit;
    printf("done, %d pulses\n", rounds);
    printf("  GPIO%d (ch B, interrupt handler): %s\n", ISR_PIN,
           saw_isr ? "seen high" : "NEVER HIGH");
    printf("  GPIO%d (ch C, woken task):        %s\n", TASK_PIN,
           saw_task ? "seen high" : "NEVER HIGH");
    if (!saw_isr || !saw_task)
        printf("  a pin that never went high means scope_demo is not the\n"
               "  loaded image, or the doorbell is not reaching it\n");
    close(fd);
    return 0;
}

// Read the layout the provisioner wrote into the device tree.
//
// This is the single source of truth for where things live. Falls back to
// the compiled defaults on a card provisioned before the node carried
// these properties, and says so, because silently using different numbers
// from the ones rivet was built with is the failure this exists to
// prevent.
static int dt_u32(const char *path, uint32_t *out) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    unsigned char b[4];
    int ok = fread(b, 1, 4, f) == 4;
    fclose(f);
    if (!ok) return 0;
    *out = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8) | b[3];        // device tree is big-endian
    return 1;
}

#define DT_NODE "/proc/device-tree/reserved-memory/rivet@30000000"

static int config_from_dt = 0;

static void read_config(void) {
    uint32_t v;
    char path[256];

    snprintf(path, sizeof path, "%s/rivet,core", DT_NODE);
    if (dt_u32(path, &v)) { RIVET_CORE_N = (int)v; config_from_dt = 1; }

    // reg is <base size>, two big-endian 32-bit cells on this platform.
    snprintf(path, sizeof path, "%s/reg", DT_NODE);
    FILE *f = fopen(path, "rb");
    if (f) {
        unsigned char b[8];
        if (fread(b, 1, 8, f) == 8) {
            RIVET_BASE = ((unsigned long)b[0] << 24) | ((unsigned long)b[1] << 16) |
                         ((unsigned long)b[2] << 8) | b[3];
            config_from_dt = 1;
        }
        fclose(f);
    }

    // Sanity-check rather than trust. A shared offset of zero would put
    // the rings on top of the image, which is exactly what happened when
    // fdtput quietly stored a 0x literal as zero: every reader then
    // waited forever on a ring that could not be there. A wrong value
    // here should be a sentence, not a hang.
    snprintf(path, sizeof path, "%s/rivet,shared-offset", DT_NODE);
    if (dt_u32(path, &v)) {
        if (v >= 0x100000 && v < 0x8000000) {
            SHMEM_BASE = RIVET_BASE + v;
            config_from_dt = 1;
        } else {
            fprintf(stderr,
                    "device tree says rivet,shared-offset = %#x, which cannot "
                    "be right; using %#lx\n", v, SHMEM_BASE);
        }
    }
    if (RIVET_CORE_N < 0 || RIVET_CORE_N > 3) {
        fprintf(stderr, "device tree says rivet,core = %d; using 3\n", RIVET_CORE_N);
        RIVET_CORE_N = 3;
    }
    if (RIVET_BASE & 0x1FFFFF) {
        fprintf(stderr, "device tree base %#lx is not 2 MiB aligned; using 0x30000000\n",
                RIVET_BASE);
        RIVET_BASE = 0x30000000UL;
    }
}

// Read the header rivet published. Returns 0 if there is none.
static int read_sysinfo(struct sysinfo_hdr *h) {
    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) return 0;
    unsigned char *p = mmap(NULL, 0x1000, PROT_READ, MAP_SHARED, fd,
                            SHMEM_BASE + SYS_OFFSET);
    close(fd);
    if (p == MAP_FAILED) return 0;

    memset(h, 0, sizeof *h);
    h->magic = *(volatile uint32_t *)(p + SYS_O_MAGIC);
    if (h->magic != SYS_MAGIC) { munmap(p, 0x1000); return 0; }
    h->abi       = *(volatile uint32_t *)(p + SYS_O_ABI);
    h->heartbeat = *(volatile uint64_t *)(p + SYS_O_HEARTBEAT);
    h->boot      = *(volatile uint64_t *)(p + SYS_O_BOOT);
    h->tick_hz   = *(volatile uint32_t *)(p + SYS_O_TICK_HZ);
    h->core      = *(volatile uint32_t *)(p + SYS_O_CORE);
    h->state     = *(volatile uint32_t *)(p + SYS_O_STATE);
    h->beat_hz   = *(volatile uint32_t *)(p + SYS_O_BEAT_HZ);
    h->load_base = *(volatile uint64_t *)(p + SYS_O_LOAD_BASE);
    h->shared    = *(volatile uint64_t *)(p + SYS_O_SHARED);
    h->owned_len = *(volatile uint64_t *)(p + SYS_O_OWNED_LEN);
    memcpy(h->sysver,   p + SYS_O_SYSVER,   32);
    memcpy(h->image,    p + SYS_O_IMAGE,    32);
    memcpy(h->build,    p + SYS_O_BUILD,    48);
    memcpy(h->rivetver, p + SYS_O_RIVETVER, 32);
    munmap(p, 0x1000);
    return 1;
}

// Refuse to interpret a header this build does not understand.
//
// The alternative is reading fields at offsets that have moved, which
// produces confident nonsense. Warn rather than exit for the commands
// that only stream bytes, since a ring is a ring whatever the header says.
static int abi_ok(const struct sysinfo_hdr *h, int fatal) {
    if (h->abi == SYS_ABI) return 1;
    fprintf(stderr,
            "%srivet image speaks header ABI %u, this tool speaks %u%s\n",
            CRED, h->abi, SYS_ABI, CRST);
    fprintf(stderr, "  the image and the loader were not built together\n");
    if (fatal) exit(1);
    return 0;
}

static void human_bytes(unsigned long n, char *out, size_t cap) {
    if (n >= 1UL << 20) snprintf(out, cap, "%lu MiB", n >> 20);
    else if (n >= 1UL << 10) snprintf(out, cap, "%lu KiB", n >> 10);
    else snprintf(out, cap, "%lu B", n);
}

static void first_line(const char *path, char *out, size_t cap) {
    out[0] = 0;
    FILE *f = fopen(path, "r");
    if (!f) return;
    if (fgets(out, (int)cap, f)) out[strcspn(out, "\n")] = 0;
    fclose(f);
}

// ── status ───────────────────────────────────────────────────────────
//
// Fields and values, in two shapes: aligned label/value pairs for scalars
// and a plain column table where there are repeated rows. Anything that
// needs a sentence to explain belongs in the documentation, not in output
// someone reads twenty times a day.

static void kv(const char *k, const char *fmt, ...) {
    va_list ap;
    printf("  %s%9s%s  ", CDIM, k, CRST);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    putchar('\n');
}

static int cmd_status(void) {
    struct sysinfo_hdr h;
    int live = read_sysinfo(&h);
    char buf[256], hb[32], hb2[32];

    if (live) {
        printf("%srivet %s%s  build %s  abi %s%u%s\n\n",
               CBOLD, h.sysver, CRST, h.build,
               h.abi == SYS_ABI ? "" : CRED, h.abi, h.abi == SYS_ABI ? "" : CRST);
        if (h.abi != SYS_ABI)
            printf("  %sabi %u unsupported, this build reads %u%s\n\n",
                   CRED, h.abi, SYS_ABI, CRST);
    } else {
        printf("%srivet%s  no image running\n\n", CBOLD, CRST);
    }

    first_line("/proc/sys/kernel/osrelease", buf, sizeof buf);
    kv("linux", "%s", buf);

    if (live) {
        const char *col = h.state == 1 ? CGRN : h.state == 2 ? CRED : CYEL;
        kv("rtos", "%s  %s", h.rivetver, h.image);
        if (h.tick_hz)
            kv("state", "%s%s%s  %u Hz tick", col, state_name(h.state), CRST, h.tick_hz);
        else
            kv("state", "%s%s%s", col, state_name(h.state), CRST);

        if (h.state != 1) {
            kv("heartbeat", "stopped");
        } else {
            struct sysinfo_hdr h2;
            usleep(300000);
            read_sysinfo(&h2);
            if (h2.heartbeat == h.heartbeat)
                kv("heartbeat", "%s%sstalled%s  none in 300 ms, expected %u",
                   CBOLD, CRED, CRST, h.beat_hz * 3 / 10);
            else
                kv("heartbeat", "%s%u Hz%s  %llu beats  %llu s", CGRN, h.beat_hz, CRST,
                   (unsigned long long)h.heartbeat,
                   h.beat_hz ? (unsigned long long)(h.heartbeat / h.beat_hz) : 0ULL);
        }
    }
    kv("config", "%s", config_from_dt ? "device tree" : "built-in defaults");

    { char t[64];
      first_line("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", t, sizeof t);
      unsigned khz = (unsigned)strtoul(t, NULL, 10);
      if (khz) kv("clock", "%u MHz", khz / 1000); }

    unsigned long memtotal = 0, memavail = 0;
    FILE *f = fopen("/proc/meminfo", "r");
    if (f) {
        while (fgets(buf, sizeof buf, f)) {
            sscanf(buf, "MemTotal: %lu kB", &memtotal);
            sscanf(buf, "MemAvailable: %lu kB", &memavail);
        }
        fclose(f);
    }
    human_bytes(memtotal * 1024, hb, sizeof hb);
    human_bytes(memavail * 1024, hb2, sizeof hb2);
    kv("memory", "%s, %s free", hb, hb2);

    long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
    printf("\n  %scpu  owner  status%s\n", CDIM, CRST);
    for (int c = 0; c < 4; c++) {
        if (live && (int)h.core == c)
            printf("  %3d  rivet  %s%s%s\n", c,
                   h.state == 1 ? CGRN : CYEL, state_name(h.state), CRST);
        else if (c < ncpu)
            printf("  %3d  linux  %sonline%s\n", c, CGRN, CRST);
        else
            printf("  %3d  %s-      parked%s\n", c, CDIM, CRST);
    }

    printf("\n  %sregion  base        size%s\n", CDIM, CRST);
    if (live) {
        human_bytes(h.owned_len, hb, sizeof hb);
        printf("  rtos    %#-11lx %s\n", (unsigned long)h.load_base, hb);
        printf("  shared  %#-11lx 2 MiB   Device-nGnRnE\n", (unsigned long)h.shared);
    } else {
        // No size column to align against here, so no padding either:
        // padding a final field just emits trailing whitespace.
        printf("  rtos    %#lx\n", RIVET_BASE);
        printf("  shared  %#lx\n", SHMEM_BASE);
    }
    printf("\n");
    return 0;
}

// One line of identity, also pushed into the kernel ring buffer so it
// lands in dmesg next to the kernel's own boot messages rather than in a
// separate log nobody correlates with them.
static int cmd_banner(void) {
    struct sysinfo_hdr h;
    char line[512];
    if (read_sysinfo(&h)) {
        snprintf(line, sizeof line,
                 "rivet: RTOS %s (%s) on core %u, %u Hz tick, %s, system %s build %s",
                 h.rivetver, h.image, h.core, h.tick_hz, state_name(h.state),
                 h.sysver, h.build);
    } else {
        snprintf(line, sizeof line, "rivet: no image running on core %d", RIVET_CORE_N);
    }
    printf("%s\n", line);
    FILE *k = fopen("/dev/kmsg", "w");
    if (k) { fprintf(k, "<5>%s\n", line); fclose(k); }
    return 0;
}

// Block until the heartbeat stops, then exit non-zero.
//
// This is what a systemd unit runs. Until it existed, a hung core and an
// idle one were indistinguishable from Linux: the console ring simply
// stopped producing, which is also what a healthy system with nothing to
// say looks like.
static int cmd_watch(int grace_ms) {
    struct sysinfo_hdr h;
    while (!read_sysinfo(&h)) {
        fprintf(stderr, "waiting for rivet to publish its header...\n");
        sleep(2);
    }
    abi_ok(&h, 0);
    printf("watching %s on core %u, %u beats/s, %d ms grace\n",
           h.image, h.core, h.beat_hz, grace_ms);
    fflush(stdout);

    uint64_t last = h.heartbeat;
    int stalled_ms = 0;
    for (;;) {
        usleep(200000);
        if (!read_sysinfo(&h)) continue;
        if (h.state != 1) {
            printf("rivet stopped: state is %s\n", state_name(h.state));
            return h.state == 2 ? 1 : 0;      // faulted is a failure, exited is not
        }
        if (h.heartbeat != last) { last = h.heartbeat; stalled_ms = 0; continue; }
        stalled_ms += 200;
        if (stalled_ms >= grace_ms) {
            fprintf(stderr, "rivet heartbeat stalled for %d ms: the core is hung\n",
                    stalled_ms);
            return 1;
        }
    }
}

int main(int argc, char **argv) {
    install_fault_handler();
    use_colour = isatty(STDOUT_FILENO) && !getenv("NO_COLOR");
    read_config();
    if (argc < 2) {
        fprintf(stderr,
                "usage: %s probe | load <image> | console | trace <file>\n"
                "       %s send <cmd> | bench [rounds] | scope [n] [ms]\n"
                "       %s status | banner | watch [grace-ms]\n",
                argv[0],
                argv[0], argv[0]);
        return 2;
    }
    if (!strcmp(argv[1], "probe"))   return cmd_probe();
    if (!strcmp(argv[1], "bench"))   return cmd_bench(argc > 2 ? atoi(argv[2]) : 500);
    if (!strcmp(argv[1], "status"))  return cmd_status();
    if (!strcmp(argv[1], "banner"))  return cmd_banner();
    if (!strcmp(argv[1], "watch"))   return cmd_watch(argc > 2 ? atoi(argv[2]) : 2000);
    if (!strcmp(argv[1], "scope"))
        return cmd_scope(argc > 2 ? atoi(argv[2]) : 200, argc > 3 ? atoi(argv[3]) : 10);
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
