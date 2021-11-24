/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2021, Hugo Lefeuvre <hugo.lefeuvre@manchester.ac.uk>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#if !LINUX_USERLAND
#include <flexos/microbenchmarks/isolated.h>
#include <flexos/isolation.h>
#include <uk/alloc.h>
#else
#include <inttypes.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

// some config checks here...
#if !LINUX_USERLAND && !CONFIG_LIBFLEXOS_GATE_INTELPKU_NO_INSTRUMENT
#error "Microbenchmarks should not be executed with gate instrumentation!"
#endif

// bench_start returns a timestamp for use to measure the start of a benchmark
// run.
__attribute__ ((always_inline)) static inline uint64_t bench_start(void)
{
  unsigned  cycles_low, cycles_high;
  asm volatile( "CPUID\n\t" // serialize
                "RDTSC\n\t" // read clock
                "MOV %%edx, %0\n\t"
                "MOV %%eax, %1\n\t"
                : "=r" (cycles_high), "=r" (cycles_low)
                :: "%rax", "%rbx", "%rcx", "%rdx" );
  return ((uint64_t) cycles_high << 32) | cycles_low;
}

// bench_end returns a timestamp for use to measure the end of a benchmark run.
__attribute__ ((always_inline)) static inline uint64_t bench_end(void)
{
  unsigned  cycles_low, cycles_high;
  asm volatile( "RDTSCP\n\t" // read clock + serialize
                "MOV %%edx, %0\n\t"
                "MOV %%eax, %1\n\t"
                "CPUID\n\t" // serialize -- but outside clock region!
                : "=r" (cycles_high), "=r" (cycles_low)
                :: "%rax", "%rbx", "%rcx", "%rdx" );
  return ((uint64_t) cycles_high << 32) | cycles_low;
}

#if !LINUX_USERLAND
/*
 * make sure function does not get inlined
 */
__attribute__ ((noinline))
void empty_fcall(void) {
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_1xB(void) {
    char characters[1];
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_1xBs(void) {
    char characters[1] __attribute__((flexos_whitelist));
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_2xB(void) {
    char characters1[1];
    char characters2[1];
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_2xBs(void) {
    char characters1[1] __attribute__((flexos_whitelist));
    char characters2[1] __attribute__((flexos_whitelist));
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_3xB(void) {
    char characters1[1];
    char characters2[1];
    char characters3[1];
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_3xBs(void) {
    char characters1[1] __attribute__((flexos_whitelist));
    char characters2[1] __attribute__((flexos_whitelist));
    char characters3[1] __attribute__((flexos_whitelist));
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_4xB(void) {
    char characters1[1];
    char characters2[1];
    char characters3[1];
    char characters4[1];
    /* keep the call from being optimized away */
    asm volatile ("");
}

__attribute__ ((noinline))
void empty_fcall_4xBs(void) {
    char characters1[1] __attribute__((flexos_whitelist));
    char characters2[1] __attribute__((flexos_whitelist));
    char characters3[1] __attribute__((flexos_whitelist));
    char characters4[1] __attribute__((flexos_whitelist));
    /* keep the call from being optimized away */
    asm volatile ("");
}
#endif

#define REPS 10000

#define SERIAL 0

static inline void RUN_ISOLATED_FCALL(void)
{
#if !LINUX_USERLAND
	flexos_gate(libflexosmicrobenchmarks, flexos_microbenchmarks_empty_fcall);
#else
	syscall(1000);
#endif
}

__attribute__ ((noinline)) void RUN_FCALL(void)
{
	asm volatile ("");
}

int main(int argc, char *argv[])
{
#if CONFIG_LIBFLEXOS_GATE_INTELPKU_PRIVATE_STACKS
    printf("Measuring gate latencies with stack isolating gates...\n");
#elif CONFIG_LIBFLEXOS_GATE_INTELPKU_SHARED_STACKS
    printf("Measuring gate latencies with shared stacks...\n");
#elif CONFIG_LIBFLEXOS_VMEPT
    printf("Measuring gate latencies with VM/EPT RPC gates...\n");
#elif LINUX_USERLAND
    printf("Measuring system call latency...\n");
#else
    printf("Measuring gate latencies with *UNKNOWN* gates...\n");
#endif

    uint32_t overhead_tsc, overhead_gate, overhead_fcall, t0, t1;

#if SERIAL
    printf("> serial\n");
    printf("TSC\tgate\tfcall\n");
    for(int i = 0; i < REPS; i++) {
        t0 = bench_start();
        asm volatile("");
        t1 = bench_end();
        overhead_tsc = t1 - t0;

        t0 = bench_start();
	RUN_ISOLATED_FCALL();
        t1 = bench_end();
        overhead_gate = t1 - t0;

        t0 = bench_start();
	RUN_FCALL();
        t1 = bench_end();
        overhead_fcall = t1 - t0;

        printf("%" PRId64 "\t%" PRId64 "\t%" PRId64 "\n", overhead_tsc,
					overhead_gate, overhead_fcall);
    }
#else
    printf("> loop\n");
    uint64_t min = 1000, max = 0, sum = 0;
    for(int i = 0; i < REPS; i++) {
        t0 = bench_start();
        asm volatile("");
        t1 = bench_end();
        if ((t1 - t0) < min) { min = (t1 - t0); }
        if ((t1 - t0) > max) { max = (t1 - t0); }
	sum += (t1 - t0);
    }

    printf("%" PRId64 "\t%" PRId64 "\t%" PRId64 "\n", sum / REPS, min, max);

    min = 1000, max = 0, sum = 0;
    for(int i = 0; i < REPS; i++) {
        t0 = bench_start();
	RUN_ISOLATED_FCALL();
        t1 = bench_end();
        if ((t1 - t0) < min) { min = (t1 - t0); }
        if ((t1 - t0) > max) { max = (t1 - t0); }
	sum += (t1 - t0);
    }

    printf("%" PRId64 "\t%" PRId64 "\t%" PRId64 "\n", sum / REPS, min, max);

    min = 1000, max = 0, sum = 0;
    for(int i = 0; i < REPS; i++) {
        t0 = bench_start();
	RUN_FCALL();
        t1 = bench_end();
        if ((t1 - t0) < min) { min = (t1 - t0); }
        if ((t1 - t0) > max) { max = (t1 - t0); }
	sum += (t1 - t0);
    }

    printf("%" PRId64 "\t%" PRId64 "\t%" PRId64 "\n", sum / REPS, min, max);
#endif

#if CONFIG_LIBFLEXOS_GATE_INTELPKU_PRIVATE_STACKS
    printf("Now measuring data sharing latencies (no domain transitions)\n");
    printf("> serial\n");

#if CONFIG_LIBFLEXOS_ENABLE_DSS
#define HEADER()					\
do {							\
    printf("data shadow stack\n");			\
    printf("TSC\tDSS\tstack\n");			\
} while(0)
#else
#define HEADER()					\
do {							\
    printf("stack-to-heap converted\n");		\
    printf("TSC\ts2h\tstack\n");			\
} while(0)
#endif

#define BENCH_NB(NB)					\
do {							\
    printf(STRINGIFY(NB) "x1B stack v.s. "		\
		STRINGIFY(NB) "x1B ");			\
    HEADER();						\
							\
    for(int i = 0; i < REPS; i++) {			\
        t0 = bench_start();				\
        asm volatile("");				\
        t1 = bench_end();				\
        overhead_tsc = t1 - t0;				\
							\
        t0 = bench_start();				\
	empty_fcall_ ## NB ## xBs();			\
        t1 = bench_end();				\
        overhead_gate = t1 - t0;			\
							\
        t0 = bench_start();				\
	empty_fcall_ ## NB ## xB();			\
        t1 = bench_end();				\
        overhead_fcall = t1 - t0;			\
							\
        printf("%" PRId64 "\t%" PRId64 "\t%"		\
		   PRId64 "\n", overhead_tsc,		\
		   overhead_gate, overhead_fcall);	\
    }							\
} while(0)

    BENCH_NB(1);
    BENCH_NB(2);
    BENCH_NB(3);
    BENCH_NB(4);

#elif !LINUX_USERLAND
    printf("Not measuring data sharing latencies as we're configured with a shared stack.\n");
#endif /* CONFIG_LIBFLEXOS_GATE_INTELPKU_PRIVATE_STACKS */

    return 0;
}
