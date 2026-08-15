// Compare the two timebases rivet ends up straddling.
//
// Deadlines are kept in `now_us`, which reads the BCM System Timer at
// 0x3f003000 and is documented as 1 MHz. Wakeups happen on the
// architected timer, CNTPCT_EL0, running at CNTFRQ_EL0 (19.2 MHz here).
// If those two drift against each other, a deadline will periodically
// fall the wrong side of a tick boundary and a wakeup lands one tick
// out, which is what the ~999 us slip every ~12 s looks like.
//
// A drift of one part in ~12000 would produce exactly that period at a
// 1 kHz tick. This measures the actual figure.
//
//   cc -O2 -o clockdrift clockdrift.c && sudo ./clockdrift 30

#define _GNU_SOURCE
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#define ST_BASE 0x3f003000UL
#define ST_CLO  0x04
#define ST_CHI  0x08

static volatile unsigned char *st;

static uint64_t sys_timer_us(void) {
    // Two 32-bit halves, so re-read the high word to catch a wrap.
    for (;;) {
        uint32_t hi0 = *(volatile uint32_t *)(st + ST_CHI);
        uint32_t lo  = *(volatile uint32_t *)(st + ST_CLO);
        uint32_t hi1 = *(volatile uint32_t *)(st + ST_CHI);
        if (hi0 == hi1) return ((uint64_t)hi1 << 32) | lo;
    }
}

// The virtual counter, not the physical one. Linux leaves
// CNTKCTL_EL1.EL0PCTEN clear, so reading CNTPCT_EL0 from userspace traps
// with SIGILL; CNTVCT_EL0 is the one exposed to EL0 for the vDSO. The
// armstub sets CNTVOFF_EL2 to zero on this board, so the two count the
// same thing, which is what makes this a valid stand-in for the counter
// rivet reads at EL1.
static uint64_t cntpct(void) {
    uint64_t v;
    __asm__ __volatile__("isb; mrs %0, cntvct_el0" : "=r"(v));
    return v;
}

static uint64_t cntfrq(void) {
    uint64_t v;
    __asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(v));
    return v;
}

int main(int argc, char **argv) {
    int secs = argc > 1 ? atoi(argv[1]) : 30;

    int fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (fd < 0) { perror("/dev/mem"); return 1; }
    void *p = mmap(NULL, 4096, PROT_READ, MAP_SHARED, fd, (off_t)ST_BASE);
    if (p == MAP_FAILED) { perror("mmap system timer"); return 1; }
    st = p;

    uint64_t f = cntfrq();
    printf("CNTFRQ_EL0        %" PRIu64 " Hz\n", f);
    printf("sampling for %d s...\n", secs);

    uint64_t a_st = sys_timer_us(), a_ct = cntpct();
    sleep(secs);
    uint64_t b_st = sys_timer_us(), b_ct = cntpct();

    uint64_t d_st = b_st - a_st;                 // microseconds, if it is 1 MHz
    uint64_t d_ct = b_ct - a_ct;                 // architected ticks
    double ct_us  = (double)d_ct * 1e6 / (double)f;

    printf("system timer      %" PRIu64 " us\n", d_st);
    printf("arch timer        %" PRIu64 " ticks = %.0f us\n", d_ct, ct_us);

    double diff = (double)d_st - ct_us;
    double ppm  = diff / ct_us * 1e6;
    printf("difference        %.0f us over %.0f us  (%.1f ppm)\n", diff, ct_us, ppm);

    // At a 1 kHz tick, a deadline slips a whole tick once the two clocks
    // have diverged by 1000 us.
    if (diff != 0.0) {
        double secs_per_slip = 1000.0 / (diff / (double)secs);
        printf("=> one 1 ms slip every %.1f s\n",
               secs_per_slip < 0 ? -secs_per_slip : secs_per_slip);
    } else {
        printf("=> no measurable drift\n");
    }
    return 0;
}
