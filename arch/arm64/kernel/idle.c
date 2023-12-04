// SPDX-License-Identifier: GPL-2.0-only
/*
 * Low-level idle sequences
 */

#include <linux/cpu.h>
#include <linux/irqflags.h>
#include <linux/tick.h>

#include <asm/barrier.h>
#include <asm/cpuidle.h>
#include <asm/cpufeature.h>
#include <asm/sysreg.h>

static DEFINE_PER_CPU(struct hrtimer, wfi_timer);
s64 teo_wfi_timeout_ns(void);

/*
 *	cpu_do_idle()
 *
 *	Idle the processor (wait for interrupt).
 *
 *	If the CPU supports priority masking we must do additional work to
 *	ensure that interrupts are not masked at the PMR (because the core will
 *	not wake up if we block the wake up signal in the interrupt controller).
 */
void noinstr cpu_do_idle(void)
{
	s64 wfi_timeout_ns = teo_wfi_timeout_ns();
	struct arm_cpuidle_irq_context context;
	struct hrtimer *timer = NULL;

	/*
	 * If the tick is stopped, arm a timer to ensure that the CPU doesn't
	 * stay in WFI too long and burn power. That way, the CPU will be woken
	 * up so it can enter a deeper idle state instead of staying in WFI.
	 */
	if (wfi_timeout_ns) {
		/* Use TEO's estimated sleep duration with some slack added */
		timer = this_cpu_ptr(&wfi_timer);
		hrtimer_start(timer, ns_to_ktime(wfi_timeout_ns),
			      HRTIMER_MODE_REL_PINNED_HARD);
	}

	arm_cpuidle_save_irq_context(&context);

	dsb(sy);
	wfi();

	arm_cpuidle_restore_irq_context(&context);

	/* Cancel the timer if it was armed. This always succeeds. */
	if (timer)
		hrtimer_try_to_cancel(timer);
}

static int __init wfi_timer_init(void)
{
	int cpu;

	/* No function is needed; the timer is canceled while IRQs are off */
	for_each_possible_cpu(cpu)
		hrtimer_init(&per_cpu(wfi_timer, cpu), CLOCK_MONOTONIC,
			     HRTIMER_MODE_REL_HARD);
	return 0;
}
pure_initcall(wfi_timer_init);

/*
 * This is our default idle handler.
 */
void noinstr arch_cpu_idle(void)
{
	/*
	 * This should do all the clock switching and wait for interrupt
	 * tricks
	 */
	cpu_do_idle();
	raw_local_irq_enable();
}
