/**
 * @file platform/systick.c
 * @brief Platform-support routines, SysTick component
 *
 * @author Alberto de Villa <alberto.de.villa@eee.upd.edu.ph>
 * @author Christian Klein C. Ramos (Changes made)
 * @date   2024-10-28
 * @lastmodified 2025-05-07
 */

/*
 * PIC32CM5164LS00048 initial configuration:
 * -- Architecture: ARMv8 Cortex-M23
 * -- GCLK_GEN0: OSC16M @ 4 MHz, no additional prescaler
 * -- Main Clock: No additional prescaling (always uses GCLK_GEN0 as input)
 * -- Mode: Secure, NONSEC disabled
 *
 * New clock configuration:
 * -- GCLK_GEN0: 24 MHz (DFLL48M [48 MHz], with /2 prescaler)
 * -- GCLK_GEN2: 4 MHz  (OSC16M @ 4 MHz, no additional prescaler)
 *
 * NOTE: This file does not deal directly with hardware configuration.
 */

// Common include for the XC32 compiler
#include <xc.h>       // For microcontroller specific definitions like SysTick
#include <stdbool.h>  // For bool type
#include <stdint.h>   // For UINT32_MAX, int64_t

#include "../platform.h" // For platform_timespec_t, PLATFORM_TIMESPEC_ZERO, PLATFORM_TICK_PERIOD_US

/////////////////////////////////////////////////////////////////////////////
// Timespec utility functions
/////////////////////////////////////////////////////////////////////////////

/**
 * @brief Normalizes a platform_timespec_t structure.
 * @details This function ensures that the nanosecond component (`nr_nsec`)
 *          is less than 1,000,000,000 by adding any excess nanoseconds
 *          to the seconds component (`nr_sec`). If `nr_sec` reaches
 *          `UINT32_MAX`, `nr_nsec` is capped at 999,999,999 to prevent
 *          overflow of `nr_sec`.
 * @param ts Pointer to the platform_timespec_t structure to be normalized.
 *           This structure is modified in place.
 * @return void
 */
void platform_timespec_normalize(platform_timespec_t *ts)
{
	// While nanoseconds are one billion or more, adjust seconds and nanoseconds.
	while (ts->nr_nsec >= 1000000000) {
		ts->nr_nsec -= 1000000000; // Subtract one second worth of nanoseconds
		if (ts->nr_sec < UINT32_MAX) {
			ts->nr_sec++; // Increment seconds
		} else {
			// Seconds field is already at maximum. Cap nanoseconds to prevent overflow.
			ts->nr_nsec = (1000000000 - 1); // Max representable nanoseconds
			break; // Exit loop as we cannot increment seconds further
		}
	}
}

/**
 * @brief Compares two platform_timespec_t timestamps.
 * @param lhs Pointer to the left-hand side platform_timespec_t structure for comparison.
 * @param rhs Pointer to the right-hand side platform_timespec_t structure for comparison.
 * @return int Returns:
 *             -1 if lhs is earlier than rhs.
 *             +1 if lhs is later than rhs.
 *              0 if lhs and rhs are equal.
 */
int platform_timespec_compare(const platform_timespec_t *lhs,
	const platform_timespec_t *rhs)
{
	if (lhs->nr_sec < rhs->nr_sec) {
		return -1; // lhs seconds is less than rhs seconds
	} else if (lhs->nr_sec > rhs->nr_sec) {
		return +1; // lhs seconds is greater than rhs seconds
	} else {
		// Seconds are equal, compare nanoseconds
		if (lhs->nr_nsec < rhs->nr_nsec) {
			return -1; // lhs nanoseconds is less than rhs nanoseconds
		} else if (lhs->nr_nsec > rhs->nr_nsec) {
			return +1; // lhs nanoseconds is greater than rhs nanoseconds
		} else {
			return 0;  // Both seconds and nanoseconds are equal
		}
	}
}

/////////////////////////////////////////////////////////////////////////////
// SysTick handling
/////////////////////////////////////////////////////////////////////////////

// Global wall clock timestamp, updated by SysTick_Handler
static volatile platform_timespec_t ts_wall = PLATFORM_TIMESPEC_ZERO;
// Cookie used for ensuring atomic reads of ts_wall
static volatile uint32_t ts_wall_cookie = 0;

/**
 * @brief SysTick Interrupt Handler.
 * @details This function is called at every SysTick interrupt. It increments
 *          the global wall clock time (`ts_wall`) by `PLATFORM_TICK_PERIOD_US`
 *          microseconds. It uses a cookie mechanism (`ts_wall_cookie`) to help
 *          detect race conditions when reading `ts_wall`. The SysTick counter
 *          (SysTick->VAL) is cleared before returning to ensure the timer
 *          reloads and continues.
 * @note This function has the `interrupt` attribute, indicating it's an ISR.
 *       The `used` attribute ensures the linker doesn't discard it.
 *       Wrap-around for `nr_sec` and `ts_wall_cookie` is intentional.
 * @return void
 */
void __attribute__((used, interrupt())) SysTick_Handler(void)
{
	// Create a local copy of the wall clock to perform updates
	platform_timespec_t current_time = ts_wall;
	
	// Add the defined tick period (converted from microseconds to nanoseconds)
	current_time.nr_nsec += (PLATFORM_TICK_PERIOD_US * 1000);
	
	// Normalize the nanoseconds and seconds
	// This is a manual normalization because platform_timespec_normalize is not designed for ISRs
	// and handles UINT32_MAX saturation differently than simple wrap-around.
	while (current_time.nr_nsec >= 1000000000) {
		current_time.nr_nsec -= 1000000000;
		current_time.nr_sec++;	// Allow nr_sec to wrap around (uint32_t behavior)
	}
	
	// Update the cookie before and after writing to ts_wall.
	// This allows readers to detect if an interrupt occurred during their read.
	ts_wall_cookie++;	// Pre-increment cookie (wrap-around is intentional)
	ts_wall = current_time; // Update the global wall clock
	ts_wall_cookie++;	// Post-increment cookie (wrap-around is intentional)
	
	// Clear the SysTick COUNTFLAG and reload the timer by writing any value to VAL.
	// This ensures the SysTick interrupt flag is cleared.
	SysTick->VAL  = 0x00158158;	// Any dummy value clears the counter and COUNTFLAG
	return;
}

/**
 * @brief SysTick Reload Value Calculation.
 * @details Defines the value to be loaded into the SysTick LOAD register.
 *          It's calculated based on a 24MHz system clock, divided by 2 for SysTick,
 *          resulting in a 12MHz SysTick clock.
 *          `PLATFORM_TICK_PERIOD_US` is the desired period in microseconds.
 * @note According to ARM documentation, the SysTick timer counts from LOAD down to 0,
 *       taking LOAD+1 cycles. For a period of N cycles, LOAD should be N-1.
 *       This macro calculates N, not N-1. If `PLATFORM_TICK_PERIOD_US` represents
 *       the exact desired period, this definition will result in a period that is
 *       one SysTick clock cycle longer than intended.
 *       For example, if `PLATFORM_TICK_PERIOD_US` is 1000us and SysTick clock is 12MHz,
 *       this calculates `LOAD = 12 * 1000 = 12000`. The timer will take 12001 cycles.
 *       The actual period will be `1000us + (1/12)us`.
 *       The `SysTick_Handler` adds exactly `PLATFORM_TICK_PERIOD_US * 1000` ns,
 *       so this discrepancy can lead to time drift.
 */
#define SYSTICK_CLK_MHZ (24/2) // SysTick clock frequency in MHz (12 MHz)
#define SYSTICK_RELOAD_VAL (SYSTICK_CLK_MHZ * PLATFORM_TICK_PERIOD_US)

/**
 * @brief Initializes the SysTick timer.
 * @details Configures the SysTick timer to generate interrupts at a frequency
 *          determined by `SYSTICK_RELOAD_VAL`.
 *          The initialization sequence follows ARM v8-M recommendations:
 *          1. Program the LOAD register.
 *          2. Clear the VAL register (current value).
 *          3. Program the CTRL register (control and status).
 * @return void
 */
void platform_systick_init(void)
{
	/*
	 * Per the Arm v8-M Architecture Reference Manual (Section D3.3.4 SysTick Programmers Model):
	 * To ensure correct operation when the SysTick counter is enabled, software must
	 * observe the following sequence for SysTick register updates:
	 * 1. Program reload value in SYST_RVR.
	 * 2. Clear current value in SYST_CVR by writing any value to it.
	 * 3. Configure and enable SysTick in SYST_CSR.
	 */

	// 1. Program the reload value.
	// Note: See comment for SYSTICK_RELOAD_VAL about potential off-by-one.
	SysTick->LOAD = SYSTICK_RELOAD_VAL;

	// 2. Clear the current SysTick counter value and the COUNTFLAG.
	// Any write to SysTick->VAL clears it.
	SysTick->VAL  = 0x00158158; // Dummy value

	// 3. Program the SysTick Control and Status Register (CTRL/CSR).
	//    Bit 0 (ENABLE): 1 = counter enabled
	//    Bit 1 (TICKINT): 1 = enable SysTick interrupt
	//    Bit 2 (CLKSOURCE): 1 = use processor clock, 0 = use external reference clock (implementation defined)
	//    Value 0x00000007 enables the counter, enables interrupts, and selects the processor clock.
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
	// SysTick->CTRL = 0x00000007; // Equivalent to the above if masks are 1, 2, 4
	return;
}

/**
 * @brief Retrieves the current wall clock time.
 * @details This function safely reads the global wall clock time (`ts_wall`),
 *          which is updated by the `SysTick_Handler`. It uses a cookie mechanism
 *          to ensure that the read value is consistent and not corrupted by an
 *          interrupt occurring mid-read. If a change is detected (cookie mismatch),
 *          the read is retried.
 * @param tick Pointer to a platform_timespec_t structure where the current
 *             wall clock time will be stored.
 * @return void
 */
void platform_tick_count(platform_timespec_t *tick)
{
	uint32_t cookie_before_read;
	
	// Loop to ensure a consistent read of ts_wall, guarded by ts_wall_cookie.
	// If ts_wall_cookie changes during the read of ts_wall, it means SysTick_Handler
	// interrupted and modified ts_wall, so we must retry the read.
	do {
		cookie_before_read = ts_wall_cookie; // Read cookie before accessing shared data
		*tick = ts_wall;                     // Read the shared data
	} while (ts_wall_cookie != cookie_before_read); // Check if cookie changed during read
}

/**
 * @brief Retrieves a high-resolution timestamp.
 * @details This function provides a more precise timestamp than `platform_tick_count`
 *          by combining the last tick count (from `platform_tick_count`) with the
 *          current value of the SysTick down-counter. This accounts for the time
 *          elapsed since the last SysTick interrupt.
 * @param tick Pointer to a platform_timespec_t structure where the
 *             high-resolution time will be stored.
 * @return void
 */
void platform_tick_hrcount(platform_timespec_t *tick)
{
	platform_timespec_t base_tick_time;
	uint32_t elapsed_systick_cycles;
	uint32_t current_systick_val;
    uint32_t cookie_before_read;

    // To get a consistent high-resolution time, we need to read both
    // the base tick time (ts_wall) and the current SysTick counter value (SysTick->VAL)
    // in a way that avoids inconsistencies if a SysTick interrupt occurs.
    // We read ts_wall using the cookie mechanism, and then read SysTick->VAL.
    // If an interrupt occurred just before reading SysTick->VAL, ts_wall would be updated,
    // and SysTick->VAL would be near its reload value. This is generally fine.
    // The platform_tick_count ensures we get a consistent base_tick_time.
    do {
        cookie_before_read = ts_wall_cookie;
        platform_tick_count(&base_tick_time); // Get the last full tick time (already handles its own cookie)
        current_systick_val = SysTick->VAL;   // Get current value of the SysTick down-counter
        // If ts_wall_cookie changed between platform_tick_count completing and reading SysTick->VAL,
        // it means an interrupt occurred. base_tick_time would be the *new* time, and
        // current_systick_val is also from the *new* tick interval. This is consistent.
        // If an interrupt occurred *within* platform_tick_count, it would retry.
        // The critical check is that ts_wall (via base_tick_time) and current_systick_val
        // are from the same "era" or that if ts_wall advanced, current_systick_val is relative to that new advance.
    } while (ts_wall_cookie != cookie_before_read && ts_wall_cookie != (cookie_before_read + 1));
    // The (cookie_before_read + 1) check is a heuristic. If only one increment happened (SysTick_Handler ran once),
    // platform_tick_count would have gotten the new time, and current_systick_val is fine.
    // A simpler approach is often to just call platform_tick_count and then read SysTick->VAL,
    // accepting the small window for a race, which usually results in a slightly old sub-tick value
    // if an interrupt hits exactly between the two operations.
    // The original code did:
    // platform_tick_count(&t);
    // uint32_t s = SYSTICK_RELOAD_VAL - SysTick->VAL;
    // This is generally robust because platform_tick_count ensures 't' is consistent.
    // If an interrupt happens after platform_tick_count but before SysTick->VAL read,
    // 't' is the new time, and SysTick->VAL is early in its new countdown. This is correct.

	// For simplicity and to match original intent's robustness level:
	platform_tick_count(&base_tick_time);
	current_systick_val = SysTick->VAL;

	// Calculate how many SysTick clock cycles have elapsed in the current tick period.
	// SysTick counts down from SYSTICK_RELOAD_VAL.
	// elapsed_systick_cycles = SYSTICK_RELOAD_VAL - current_systick_val.
	// Note: If SYSTICK_RELOAD_VAL is N (for N+1 cycles), this is correct.
	// If SYSTICK_RELOAD_VAL should be N-1 (for N cycles), this is also correct
	// as it measures cycles from the loaded value.
	elapsed_systick_cycles = SYSTICK_RELOAD_VAL - current_systick_val;
	
	// Add the elapsed sub-tick time (in nanoseconds) to the base tick time.
	// Each SysTick cycle corresponds to (1 / SYSTICK_CLK_MHZ) microseconds.
	// Nanoseconds = elapsed_systick_cycles * (1000 / SYSTICK_CLK_MHZ).
	base_tick_time.nr_nsec += (elapsed_systick_cycles * 1000) / SYSTICK_CLK_MHZ;
	
	// Normalize the nanoseconds and seconds (similar to SysTick_Handler)
	// This handles the case where adding sub-tick nanoseconds causes nr_nsec to exceed 1 billion.
	while (base_tick_time.nr_nsec >= 1000000000) {
		base_tick_time.nr_nsec -= 1000000000;
		base_tick_time.nr_sec++;	// Allow nr_sec to wrap around
	}
	
	*tick = base_tick_time; // Store the calculated high-resolution time
}

/**
 * @brief Calculates the difference between two timestamps (lhs - rhs).
 * @details This function computes the duration from an earlier time (`rhs`)
 *          to a later time (`lhs`). It correctly handles wrap-around of the
 *          `nr_sec` (seconds) counter, assuming `lhs` is chronologically
 *          after `rhs` even if `lhs->nr_sec` is numerically smaller due to
 *          a wrap-around. The resulting difference is stored in `diff`.
 * @param diff Pointer to a platform_timespec_t structure where the calculated
 *             difference will be stored.
 * @param lhs Pointer to the minuend platform_timespec_t (later timestamp).
 * @param rhs Pointer to the subtrahend platform_timespec_t (earlier timestamp).
 * @return void
 */
void platform_tick_delta(
	platform_timespec_t *diff,
	const platform_timespec_t *lhs, const platform_timespec_t *rhs
	)
{
	uint32_t result_sec;
	int64_t result_nsec_signed; // Use signed 64-bit for nanosecond intermediate calculation

	// Calculate the difference in seconds, accounting for wrap-around of lhs->nr_sec.
	// If lhs->nr_sec < rhs->nr_sec, it means the seconds counter for lhs has wrapped around
	// relative to rhs.
	if (lhs->nr_sec >= rhs->nr_sec) {
		result_sec = lhs->nr_sec - rhs->nr_sec;
	} else {
		// lhs->nr_sec has wrapped. Calculate difference across UINT32_MAX.
		// (UINT32_MAX - rhs->nr_sec) is time from rhs to wrap.
		// Then add lhs->nr_sec (time from wrap to lhs) and 1 (for the wrap itself).
		result_sec = (UINT32_MAX - rhs->nr_sec) + lhs->nr_sec + 1;
	}

	// Calculate the difference in nanoseconds.
	result_nsec_signed = (int64_t)lhs->nr_nsec - (int64_t)rhs->nr_nsec;

	// Check if a borrow from the seconds component is needed.
	if (result_nsec_signed < 0) {
		result_nsec_signed += 1000000000; // Borrow one second (in nanoseconds)
		if (result_sec == 0) {
			// Borrowing from 0 seconds means the actual duration wraps around UINT32_MAX seconds.
			// This implies lhs and rhs were very close, straddling a second boundary,
			// and the overall second difference was 0 before borrow, but now needs to show
			// almost a full UINT32_MAX cycle if interpreted as "time until next event"
			// or if lhs was indeed (0, small_ns) and rhs was (UINT32_MAX, large_ns).
			result_sec = UINT32_MAX; // Wraps around to max value
		} else {
			result_sec--; // Decrement seconds
		}
	}

	diff->nr_sec = result_sec;
	diff->nr_nsec = (uint32_t)result_nsec_signed; // Cast is safe as value is in [0, 999999999]

	// The result `diff` is now a duration.
	// No further normalization like platform_timespec_normalize is typically needed for a delta,
	// as nr_nsec is already in the correct range [0, 999,999,999] and nr_sec represents
	// the total full seconds of the duration, which can be large.
	// The original code had a comment "Normalize..." but did not call a normalize function.
	return;
}