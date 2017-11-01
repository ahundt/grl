/// MacOS scheduler functions
#ifndef _GRL_REALTIME_HPP_
#define _GRL_REALTIME_HPP_

#ifdef __APPLE__
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/mach_init.h>
#include <mach/thread_policy.h>
//#include <mach/sched.h>
#include <pthread.h>
#include <unistd.h>
#include <err.h>
#include <sys/param.h>


static inline uint64_t
nanos_to_abs(uint64_t ns, uint32_t numer, uint32_t denom)
{
	return (uint64_t)(ns * (((double)denom) / ((double)numer)));
}


/**
 * THREAD_TIME_CONSTRAINT_POLICY:
 *
 * This scheduling mode is for threads which have real time
 * constraints on their execution.
 *
 * Parameters:
 *
 * @param period: This is the nominal amount of time between separate
 * processing arrivals, specified in absolute time units.  A
 * value of 0 indicates that there is no inherent periodicity in
 * the computation. (nanoseconds)
 *
 * @param computation: This is the nominal amount of computation
 * time needed during a separate processing arrival, specified
 * in absolute time units. (nanoseconds)
 *
 * @param constraint: This is the maximum amount of real time that
 * may elapse from the start of a separate processing arrival
 * to the end of computation for logically correct functioning,
 * specified in absolute time units.  Must be (>= computation).
 * Note that latency = (constraint - computation). (nanoseconds)
 *
 * @param preemptible: This indicates that the computation may be
 * interrupted, subject to the constraint specified above
 * Should almost always be false unless you really need it. (bool)
 *
 * @see http://www.opensource.apple.com/source/xnu/xnu-2050.18.24/tools/tests/xnu_quick_test/sched_tests.c
 * @see https://developer.apple.com/library/mac/documentation/Darwin/Conceptual/KernelProgramming/scheduler/scheduler.html
 */
inline int set_realtime(int period, int computation, int constraint, bool preemptible = false) {

	struct mach_timebase_info mti;
    struct thread_time_constraint_policy ttcpolicy;
	kern_return_t kret;

	kret = mach_timebase_info(&mti);

	if (kret != KERN_SUCCESS) {
		warnx("Could not get timebase info %d", kret);
		return 0;
	}

    thread_port_t threadport = pthread_mach_thread_np(pthread_self());

    ttcpolicy.period      = nanos_to_abs(period, mti.numer, mti.denom);
    ttcpolicy.computation = nanos_to_abs(computation, mti.numer, mti.denom); // HZ/3300;
    ttcpolicy.constraint  = nanos_to_abs(constraint, mti.numer, mti.denom); // HZ/2200;
    ttcpolicy.preemptible = preemptible;

    if ((kret=thread_policy_set(threadport,
        THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&ttcpolicy,
        THREAD_TIME_CONSTRAINT_POLICY_COUNT)) != KERN_SUCCESS) {
            fprintf(stderr, "set_realtime() failed.\n");
		    warnx("Failed to set_realtime %d", kret);

            return 0;
    }
    return 1;
}
#endif // __APPLE__




#endif