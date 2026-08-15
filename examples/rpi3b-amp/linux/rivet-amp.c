// Linux-side tool for running rivet on a core Linux was told not to use.
//
//   rivet-amp probe                 report whether this machine is set up
//   rivet-amp load <image>          copy the image in and release the core
//   rivet-amp console               drain rivet's shared-memory ring
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
#define RING_MAGIC   0x52565443U    // "RVTC"
#define OFF_MAGIC    0
#define OFF_CAP      8
#define OFF_WRITE    16
#define OFF_READ     24
#define OFF_DATA     32

// The firmware's spin table: one 64-bit slot per core.
#define SPIN_TABLE   0xd8UL
#define RIVET_CORE   3

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

    void *r = map_phys(fd, RIVET_BASE, 0x1000, 1);
    printf("map reserved %#lx : %s\n", RIVET_BASE, r ? "OK" : "FAILED (is mem= set?)");
    if (!r) rc = 1; else munmap(r, 0x1000);

    void *s = map_phys(fd, SHMEM_BASE, 0x1000, 1);
    printf("map shared   %#lx : %s\n", SHMEM_BASE, s ? "OK" : "FAILED");
    if (!s) rc = 1; else munmap(s, 0x1000);

    void *t = map_phys(fd, 0, 0x1000, 1);
    printf("map spintable %#lx      : %s\n", SPIN_TABLE,
           t ? "OK" : "FAILED (needs the kernel-module fallback)");
    if (!t) rc = 1; else munmap(t, 0x1000);

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

    unsigned long span = ((unsigned long)sz + 0xFFFF) & ~0xFFFFUL;
    unsigned char *dst = map_phys(fd, RIVET_BASE, span, 1);
    if (!dst) { perror("mmap reserved region"); close(fd); fclose(f); return 1; }

    if (fread(dst, 1, (size_t)sz, f) != (size_t)sz) {
        fprintf(stderr, "short read of image\n");
        close(fd); fclose(f); return 1;
    }
    fclose(f);
    printf("loaded %ld bytes at %#lx\n", sz, RIVET_BASE);

    // Clear the ring header so `console` does not replay a previous run's
    // output as if it were new.
    unsigned char *ring = map_phys(fd, SHMEM_BASE, 0x1000, 1);
    if (ring) {
        memset(ring, 0, 64);
        msync(ring, 64, MS_SYNC);
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

static int cmd_console(void) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("/dev/mem"); return 1; }
    unsigned char *ring = map_phys(fd, SHMEM_BASE, SHMEM_LEN, 1);
    if (!ring) { perror("mmap shared window"); close(fd); return 1; }

    volatile uint32_t *magic = (volatile uint32_t *)(ring + OFF_MAGIC);
    volatile uint32_t *cap   = (volatile uint32_t *)(ring + OFF_CAP);
    volatile uint64_t *wp    = (volatile uint64_t *)(ring + OFF_WRITE);
    volatile uint64_t *rp    = (volatile uint64_t *)(ring + OFF_READ);

    fprintf(stderr, "waiting for the ring at %#lx...\n", SHMEM_BASE);
    while (*magic != RING_MAGIC) usleep(20000);
    uint32_t capacity = *cap;
    fprintf(stderr, "ring live, capacity %u bytes. Ctrl-C to stop.\n", capacity);

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
            putchar(ring[OFF_DATA + (r % capacity)]);
            r++;
        }
        fflush(stdout);
        *rp = r;
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s probe | load <image> | console\n", argv[0]);
        return 2;
    }
    if (!strcmp(argv[1], "probe"))   return cmd_probe();
    if (!strcmp(argv[1], "console")) return cmd_console();
    if (!strcmp(argv[1], "load")) {
        if (argc < 3) { fprintf(stderr, "load needs an image path\n"); return 2; }
        return cmd_load(argv[2]);
    }
    fprintf(stderr, "unknown command %s\n", argv[1]);
    return 2;
}
