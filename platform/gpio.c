/**
 * @file      platform/gpio.c
 * @brief     Platform-support routines, GPIO component, and initialization entrypoints for PIC32CM5164LS00048.
 * @author    Alberto de Villa <alberto.de.villa@eee.upd.edu.ph>
 * @author    Christian Klein C. Ramos (Changes made)
 * @date      07 May 2025
 * @note      Enhanced Doxygen documentation and comments for clarity.
 *            Variable names, values, and core logic are preserved as per original.
 *            Updated author information.
 *
 * @details
 *            PIC32CM5164LS00048 initial configuration:
 *            -- Architecture: ARMv8 Cortex-M23
 *            -- GCLK_GEN0: OSC16M @ 4 MHz, no additional prescaler
 *            -- Main Clock: No additional prescaling (always uses GCLK_GEN0 as input)
 *            -- Mode: Secure, NONSEC disabled
 *
 *            New clock configuration implemented by this module:
 *            -- GCLK_GEN0: 24 MHz (DFLL48M [48 MHz], with /2 prescaler) for high-speed operations.
 *            -- GCLK_GEN2: 4 MHz  (OSC16M @ 4 MHz, no additional prescaler) for slower peripherals.
 *
 *            Hardware configuration for the corresponding Curiosity Nano+ Touch Evaluation Board:
 *            -- PA15: Active-HIGH LED (On-board LED)
 *            -- PA23: Active-LOW Pushbutton (On-board SW0) with external pull-up, connected to EIC_EXTINT[2].
 */

// Common include for the XC32 compiler, which includes device-specific headers
#include <xc.h>
#include <stdbool.h>
#include <string.h> // For memset, if used (not in current code, but good practice if manipulating structs)

#include "../platform.h" // Assuming this contains PLATFORM_GPO_LED_ONBOARD etc.

// Initializers defined in other platform/*.c files
extern void platform_systick_init(void);

extern void platform_usart_init(void);
extern void platform_usart_tick_handler(const platform_timespec_t *tick);

extern void gps_platform_usart_init(void);
extern void gps_platform_usart_tick_handler(const platform_timespec_t *tick);

/////////////////////////////////////////////////////////////////////////////
// Clock and Performance Level Configuration
/////////////////////////////////////////////////////////////////////////////

/**
 * @brief Configures the system clocks and performance level for higher performance.
 *
 * This function performs the following steps:
 * 1. Switches the MCU from Performance Level 0 (PL0) to Performance Level 2 (PL2)
 *    to allow for higher operating frequencies.
 * 2. Sets the Non-Volatile Memory (NVM) controller read wait states appropriate for PL2.
 * 3. Powers up and configures the 48MHz Digital Frequency Locked Loop (DFLL).
 *    This includes loading calibration values from fuses.
 * 4. Configures Generic Clock Generator 2 (GCLK_GEN2) to output 4 MHz, sourced from OSC16M.
 * 5. Configures Generic Clock Generator 0 (GCLK_GEN0) to output 24 MHz, sourced from the
 *    48MHz DFLL with a division factor of 2. This becomes the main system clock.
 *
 * @note The literal values used for register configuration are specific to the
 *       PIC32CM5164LS00048 and the target clock setup.
 */
static void raise_perf_level(void)
{
	uint32_t tmp_reg = 0; // Temporary variable for multi-step register value construction
	
	/*
	 * The chip starts in PL0, which emphasizes energy efficiency over
	 * performance. However, we need the latter for the clock frequency
	 * we will be using (~24 MHz); hence, switch to PL2 before continuing.
	 */
	// Clear the Performance Level Switch interrupt flag (PLEVELS, bit 0) in PM_INTFLAG
	PM_REGS->PM_INTFLAG = 0x01;
	// Set Performance Level Select (PLSEL) in PM_PLCFG to PL2 (value 0x02)
	PM_REGS->PM_PLCFG = 0x02;
	// Wait until the PLEVELS interrupt flag (bit 0) in PM_INTFLAG is set, indicating completion
	while ((PM_REGS->PM_INTFLAG & 0x01) == 0)
		asm("nop"); // No operation, just wait
	// Clear the PLEVELS interrupt flag again
	PM_REGS->PM_INTFLAG = 0x01;
	
	/*
	 * Power up the 48MHz DFPLL.
	 * 
	 * On the Curiosity Nano Board, VDDPLL has a 1.1uF capacitance
	 * connected in parallel. Assuming a ~20% error, we have
	 * STARTUP >= (1.32uF)/(1uF) = 1.32; as this is not an integer, choose
	 * the next HIGHER value.
	 */
	// Configure NVM Controller: Set Read Wait States (RWS) in NVMCTRL_CTRLB.
	// (2 << 1) sets RWS to 2 (value 0b10 at bit position 1).
	NVMCTRL_SEC_REGS->NVMCTRL_CTRLB = (2 << 1) ;
	// Configure Supply Controller PLL Voltage Regulator (SUPC_VREGPLL).
	// 0x00000302: Enables VREGPLL (bit 1 set) and sets STARTUP time (bits 11:8 = 3).
	SUPC_REGS->SUPC_VREGPLL = 0x00000302;
	// Wait for PLL Voltage Regulator Ready (VREGRDYPLL, bit 18) in SUPC_STATUS
	while ((SUPC_REGS->SUPC_STATUS & (1 << 18)) == 0)
		asm("nop");
	
	/*
	 * Configure the 48MHz DFPLL.
	 * 
	 * Start with disabling ONDEMAND...
	 */
	// Disable DFLL and ONDEMAND mode in OSCCTRL_DFLLCTRL by writing 0.
	OSCCTRL_REGS->OSCCTRL_DFLLCTRL = 0x0000;
	// Wait for DFLL Ready (DFLLRDY, bit 24) in OSCCTRL_STATUS
	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
		asm("nop");
	
	/*
	 * ... then writing the calibration values (which MUST be done as a
	 * single write, hence the use of a temporary variable)...
	 */
	// Read DFLL48M Coarse Calibration value from fuses (address 0x00806020)
	tmp_reg  = *((uint32_t*)0x00806020);
	// Mask the Coarse Calibration bits (6 bits starting at bit 25 of the fuse value)
	tmp_reg &= ((uint32_t)(0b111111) << 25);
	// Shift the Coarse value to align with the COARSE field in OSCCTRL_DFLLVAL (15 bit right shift)
	tmp_reg >>= 15;
	// Combine with Fine Calibration value (set to 512, masked to 10 bits)
	tmp_reg |= ((512 << 0) & 0x000003ff);
	// Write the combined Coarse and Fine calibration values to OSCCTRL_DFLLVAL
	OSCCTRL_REGS->OSCCTRL_DFLLVAL = tmp_reg;
	// Wait for DFLL Ready (DFLLRDY, bit 24) in OSCCTRL_STATUS
	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
		asm("nop");
	
	// ... then enabling ...
	// Enable the DFLL by setting the ENABLE bit (bit 1) in OSCCTRL_DFLLCTRL
	OSCCTRL_REGS->OSCCTRL_DFLLCTRL |= 0x0002;
	// Wait for DFLL Ready (DFLLRDY, bit 24) in OSCCTRL_STATUS
	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
		asm("nop");
	
	// ... then restoring ONDEMAND. (This section is commented out in the original code)
//	OSCCTRL_REGS->OSCCTRL_DFLLCTRL |= 0x0080; // Set ONDEMAND bit (bit 7)
//	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
//		asm("nop");
	
	/*
	 * Configure GCLK_GEN2 as described; this one will become the main
	 * clock for slow/medium-speed peripherals, as GCLK_GEN0 will be
	 * stepped up for 24 MHz operation.
	 */
	// Configure Generic Clock Generator 2 (GCLK_GENCTRL[2]).
	// 0x00000105: Enable generator (GENEN, bit 8), Source is OSC16M (SRC bits 4:0 = 0x05). No division. Output = 4MHz.
	GCLK_REGS->GCLK_GENCTRL[2] = 0x00000105;
	// Wait for GCLK_GENCTRL[2] synchronization to complete (SYNCBUSY bit 4 for GENCTRL2)
	while ((GCLK_REGS->GCLK_SYNCBUSY & (1 << 4)) != 0)
		asm("nop");
	
	// Switch over GCLK_GEN0 to DFLL48M, with DIV=2 to get 24 MHz.
	// Configure Generic Clock Generator 0 (GCLK_GENCTRL[0]).
	// 0x00020107: Division factor DIV = 2 (bits 26:16), Enable generator (GENEN, bit 8), Source is DFLL48M (SRC bits 4:0 = 0x07).
	// Output = 48MHz / 2 = 24MHz.
	GCLK_REGS->GCLK_GENCTRL[0] = 0x00020107;
	// Wait for GCLK_GENCTRL[0] synchronization to complete (SYNCBUSY bit 2 for GENCTRL0)
	while ((GCLK_REGS->GCLK_SYNCBUSY & (1 << 2)) != 0)
		asm("nop");
	
	// Done. We're now at 24 MHz.
	return;
}

/**
 * @brief Performs early initialization of the External Interrupt Controller (EIC).
 *
 * This function sets up the EIC before it is fully enabled. It involves:
 * 1. Configuring the clock source (GCLK_EIC) for the EIC, derived from GCLK_GEN2 (4MHz).
 * 2. Resetting the EIC peripheral to its default state.
 * 3. Configuring the debounce prescaler for EIC inputs.
 *
 * @note EIC initialization is split into "early" and "late" halves because most
 *       settings within the peripheral cannot be modified while EIC is enabled.
 *       The APB clock for EIC is assumed to be enabled by default.
 */
static void EIC_init_early(void)
{
	/*
	 * Enable the APB clock for this peripheral
	 * 
	 * NOTE: The chip resets with it enabled; hence, commented-out.
	 * 
	 * WARNING: Incorrect MCLK settings can cause system lockup that can
	 *          only be rectified via a hardware reset/power-cycle.
	 */
	// MCLK_REGS->MCLK_APBAMASK |= (1 << 10); // Example for enabling EIC clock on APBA bus if needed
	
	/*
	 * In order for debouncing to work, GCLK_EIC needs to be configured.
	 * We can pluck this off GCLK_GEN2, configured for 4 MHz; then, for
	 * mechanical inputs we slow it down to around 15.625 kHz. This
	 * prescaling is OK for such inputs since debouncing is only employed
	 * on inputs connected to mechanical switches, not on those coming from
	 * other (electronic) circuits.
	 * 
	 * GCLK_EIC is at index 4; and Generator 2 is used.
	 */
	// Configure GCLK Peripheral Channel Control for EIC (GCLK_PCHCTRL[4]).
	// 0x00000042: Enable channel (CHEN, bit 6), Select GCLK Generator 2 (GEN bits 3:0 = 0x02).
	GCLK_REGS->GCLK_PCHCTRL[4] = 0x00000042;
	// Wait until the configuration is written and channel is enabled.
	// Original check was `(GCLK_REGS->GCLK_PCHCTRL[4] & 0x00000042) == 0`.
	// This waits if the readback is not exactly 0x42. A more robust check might be for CHEN bit.
	while ((GCLK_REGS->GCLK_PCHCTRL[4] & 0x00000042) == 0) // Wait if not yet configured as 0x42
		asm("nop");
	
	// Reset, and wait for said operation to complete.
	// Reset the EIC peripheral by setting SWRST bit (bit 0) in EIC_CTRLA.
	EIC_SEC_REGS->EIC_CTRLA = 0x01;
	// Wait for Software Reset synchronization to complete (SYNCBUSY bit 0 for SWRST)
	while ((EIC_SEC_REGS->EIC_SYNCBUSY & 0x01) != 0)
		asm("nop");
	
	/*
	 * Just set the debounce prescaler for now, and leave the EIC disabled.
	 * This is because most settings are not editable while the peripheral
	 * is enabled.
	 */
	// Configure EIC Debounce Prescaler (EIC_DPRESCALER).
	// (0b0 << 16): SAMPLEN (Sample Length) = 0.
	// (0b0000 << 4): TICKON (Debounce Tick On GCLK_EIC) = 0 (uses prescaled clock).
	// (0b1111 << 0): PRESCALER0 (Prescaler value) = 15.
	// Debounce clock = GCLK_EIC / (2^(PRESCALER0+1)) = 4MHz / (2^16) = ~61 Hz.
	EIC_SEC_REGS->EIC_DPRESCALER = (0b0 << 16) | (0b0000 << 4) |
		                       (0b1111 << 0);
	return;
}

/**
 * @brief Performs late initialization of the External Interrupt Controller (EIC).
 *
 * This function enables the EIC peripheral. It should be called after all other
 * EIC configurations (like pin sense, filtering, and debouncing for specific pins)
 * have been set up.
 *
 * @note Once the EIC is enabled (EIC_CTRLA.ENABLE=1), most other configuration
 *       registers become locked or require the EIC to be disabled for modification.
 */
static void EIC_init_late(void)
{
	/*
	 * Enable the peripheral.
	 * 
	 * Once the peripheral is enabled, further configuration is almost
	 * impossible.
	 */
	// Enable the EIC by setting the ENABLE bit (bit 1) in EIC_CTRLA.
	EIC_SEC_REGS->EIC_CTRLA |= 0x02;
	// Wait for Enable synchronization to complete (SYNCBUSY bit 1 for ENABLE)
	while ((EIC_SEC_REGS->EIC_SYNCBUSY & 0x02) != 0)
		asm("nop");
	return;
}

/**
 * @brief Initializes the Event System (EVSYS).
 *
 * This function performs a software reset of the EVSYS peripheral to ensure it
 * starts in a known, default state. The APB clock for EVSYS is typically
 * enabled by default after a system reset.
 *
 * @note Actual event routing (connecting event generators to event users) is
 *       configured separately as needed by the application.
 */
static void EVSYS_init(void)
{
	/*
	 * Enable the APB clock for this peripheral
	 * 
	 * NOTE: The chip resets with it enabled; hence, commented-out.
	 * 
	 * WARNING: Incorrect MCLK settings can cause system lockup that can
	 *          only be rectified via a hardware reset/power-cycle.
	 */
	// MCLK_REGS->MCLK_APBAMASK |= (1 << 0); // Example for enabling EVSYS clock on APBA bus if needed
	
	/*
	 * EVSYS is always enabled, but may be in an inconsistent state. As
	 * such, trigger a reset.
	 */
	// Reset the EVSYS peripheral by setting SWRST bit (bit 0) in EVSYS_CTRLA.
	EVSYS_SEC_REGS->EVSYS_CTRLA = 0x01;
	// Short delay for the reset to propagate as EVSYS SWRST doesn't have a SYNCBUSY bit.
	asm("nop");
	asm("nop");
	asm("nop");
	return;
}

//////////////////////////////////////////////////////////////////////////////
// General Purpose Output (GPO) Management
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes General Purpose Outputs (GPOs).
 *
 * This function configures the on-board LED (connected to PA15) as an output pin.
 * The LED is initially set to the OFF state (output low for an Active-HIGH LED).
 *
 * @note PORT I/O configuration is hardware-dependent. This function is specific
 *       to the pin mapping of PA15 for the on-board LED as described in the file header.
 */
static void GPO_init(void)
{
	// On-board LED (PA15)
	// Clear the output latch for PA15 (turn LED OFF, as it's Active-HIGH). (1 << 15) is the mask for PA15.
	PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1 << 15);
	// Set the direction of PA15 to output.
	PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1 << 15);
	// Configure PA15 pin: Disable input buffer (INEN=0), disable pull resistor (PULLEN=0),
	// disable peripheral multiplexing (PMUXEN=0), set drive strength to normal (DRVSTR=0). Value 0x00.
	PORT_SEC_REGS->GROUP[0].PORT_PINCFG[15] = 0x00;
	
	// Done
	return;
}
	
/**
 * @brief Modifies the state of General Purpose Outputs (GPOs), specifically LEDs.
 *
 * This function allows turning specified GPOs (identified by @ref PLATFORM_GPO_LED_ONBOARD)
 * ON or OFF using bitmasks.
 *
 * @param[in] set A bitmask where each set bit corresponds to a GPO to turn ON.
 *                Use @ref PLATFORM_GPO_LED_ONBOARD for the on-board LED.
 *                Set to zero if no GPOs are to be turned ON.
 * @param[in] clr A bitmask where each set bit corresponds to a GPO to turn OFF.
 *                Use @ref PLATFORM_GPO_LED_ONBOARD for the on-board LED.
 *                Set to zero if no GPOs are to be turned OFF.
 *
 * @note If the same GPO is specified in both @p set and @p clr parameters,
 *       the CLEAR operation (turning OFF) takes precedence.
 *       The arrays `p_s` and `p_c` are used to build port-wide masks, though only
 *       GROUP[0] is used here for PA15.
 */
void platform_gpo_modify(uint16_t set, uint16_t clr)
{
	// Arrays to hold set/clear masks for multiple port groups if needed, initialized to zero.
	// p_s for setting bits (turn ON), p_c for clearing bits (turn OFF).
	uint32_t p_s[2] = {0, 0};
	uint32_t p_c[2] = {0, 0};
	
	// CLR overrides SET: if a bit is in clr, remove it from set.
	set &= ~(clr);
	
	// SET first...
	// If the on-board LED flag is in the 'set' mask, prepare to set the PA15 bit.
	if ((set & PLATFORM_GPO_LED_ONBOARD) != 0)
		p_s[0] |= (1 << 15); // Set bit 15 in the mask for PORT group 0.
	
	// ... then CLR.
	// If the on-board LED flag is in the 'clr' mask, prepare to clear the PA15 bit.
	if ((clr & PLATFORM_GPO_LED_ONBOARD) != 0)
		p_c[0] |= (1 << 15); // Set bit 15 in the mask for PORT group 0.
	
	// Commit the changes to the hardware registers.
	// PORT_OUTSET sets pins high corresponding to set bits in p_s[0].
	PORT_SEC_REGS->GROUP[0].PORT_OUTSET = p_s[0];
	// PORT_OUTCLR sets pins low corresponding to set bits in p_c[0].
	PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = p_c[0];
	return;
}

//////////////////////////////////////////////////////////////////////////////
// Pushbutton (PB) Input Management
//////////////////////////////////////////////////////////////////////////////

/*
 * Per the datasheet for the PIC32CM5164LS00048, PA23 belongs to EXTINT[2],
 * which in turn is Peripheral Function A. The corresponding Interrupt ReQuest
 * (IRQ) handler is thus named EIC_EXTINT_2_Handler.
 */
// Volatile variable to store the pushbutton event(s) mask. Modified by ISR, read by application.
static volatile uint16_t pb_press_mask = 0;

/**
 * @brief Interrupt Service Routine (ISR) for External Interrupt 2 (EIC_EXTINT[2]).
 *
 * This ISR is triggered by edge events on the pin connected to EIC_EXTINT[2] (PA23,
 * the on-board pushbutton SW0). It determines if the event was a press (pin low)
 * or a release (pin high) and updates the @c pb_press_mask accordingly with
 * @ref PLATFORM_PB_ONBOARD_PRESS or @ref PLATFORM_PB_ONBOARD_RELEASE.
 *
 * @note This function has the `interrupt` attribute, indicating it's an ISR.
 *       The `used` attribute ensures the linker doesn't discard it.
 *       The EIC must be configured for PA23 to trigger EXTINT[2].
 *       PA23 is Active-LOW (pressed = low).
 */
void __attribute__((used, interrupt())) EIC_EXTINT_2_Handler(void)
{
	// Clear any previous press/release state for the on-board button from the mask.
	pb_press_mask &= ~PLATFORM_PB_ONBOARD_MASK;

	// Check the current state of the EIC input pin for EXTINT[2] (bit 2 of EIC_PINSTATE).
	// If bit 2 is 0, the pin is low (button pressed for Active-LO configuration).
	if ((EIC_SEC_REGS->EIC_PINSTATE & (1 << 2)) == 0)
		pb_press_mask |= PLATFORM_PB_ONBOARD_PRESS; // Set the press flag.
	else // Pin is high (button released).
		pb_press_mask |= PLATFORM_PB_ONBOARD_RELEASE; // Set the release flag.
	
	// Clear the interrupt flag for EXTINT[2] (bit 2 of EIC_INTFLAG) before returning.
	// This is crucial to allow future interrupts from this source.
	EIC_SEC_REGS->EIC_INTFLAG = (1 << 2);
	return;
}

/**
 * @brief Initializes the Pushbutton (PB) input.
 *
 * This function configures PA23 (on-board pushbutton SW0) as an input.
 * It then configures the External Interrupt Controller (EIC) for PA23 (which maps
 * to EXTINT[2]) to:
 * 1. Use peripheral function A (EIC).
 * 2. Enable debouncing on the input.
 * 3. Detect both rising and falling edges (press and release).
 * 4. Enable the interrupt for EXTINT[2] at the EIC level.
 *
 * @note PORT I/O and EIC configuration is hardware-specific. This function targets
 *       PA23 for the on-board pushbutton. EIC must have undergone `EIC_init_early()`
 *       before this function is called. Global interrupts are enabled later by `NVIC_init()`.
 */
static void PB_init(void)
{
	/*
	 * Configure PA23.
	 * 
	 * NOTE: PORT I/O configuration is never separable from the in-circuit
	 *       wiring. Refer to the top of this source file for each PORT
	 *       pin assignments.
	 */
	// Set PA23 (bit 23) direction to input. 0x00800000 is (1 << 23).
	PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = 0x00800000;
	// Configure PA23 pin: Enable peripheral multiplexing (PMUXEN=1, bit 0) and
	// enable input buffer (INEN=1, bit 1). Value 0x03. (External pull-up is used).
	PORT_SEC_REGS->GROUP[0].PORT_PINCFG[23] = 0x03;
	// Select Peripheral Function A (EIC) for PA23. PA23 is an odd pin, uses PMUXO field (bits 7:4)
	// in PORT_PMUX register for its pair (PORT_PMUX[23 >> 1] = PORT_PMUX[11]).
	// Clearing PMUXO (bits 7:4) to 0x0 selects Function A. ~(0xF0) clears these bits.
	PORT_SEC_REGS->GROUP[0].PORT_PMUX[(23 >> 1)] &= ~(0xF0);
	
	/*
	 * Debounce EIC_EXT2, where PA23 is.
	 * 
	 * Configure the line for edge-detection only.
	 * 
	 * NOTE: EIC has been reset and pre-configured by the time this
	 *       function is called.
	 */
	// Enable debouncing for EXTINT[2] (bit 2 of EIC_DEBOUNCEN).
	EIC_SEC_REGS->EIC_DEBOUNCEN |= (1 << 2);
	// Configure EIC_CONFIG[0] for EXTINT[2] (SENSE2 field, bits 11:8).
	// First, clear the SENSE2 field (mask is 0xF at bit position 8).
	EIC_SEC_REGS->EIC_CONFIG0   &= ~((uint32_t)(0xF) << 8);
	// Then, set SENSE2 to 0xB (binary 1011), which typically means "both edges, filtered".
	EIC_SEC_REGS->EIC_CONFIG0   |=  ((uint32_t)(0xB) << 8);
	
	/*
	 * NOTE: Even though interrupts are enabled here, global interrupts
	 *       still need to be enabled via NVIC.
	 */
	// Enable interrupt for EXTINT[2] (bit 2 of EIC_INTENSET). Value 0x00000004.
	EIC_SEC_REGS->EIC_INTENSET = 0x00000004;
	return;
}

/**
 * @brief Retrieves the current pushbutton event mask.
 *
 * This function returns a bitmask of @c PLATFORM_PB_* values indicating which
 * pushbutton events (press and/or release) have occurred since the last call
 * to this function. The internal event mask (@c pb_press_mask) is cleared
 * after its value is read to ensure new events are captured for the next call.
 *
 * @return uint16_t A bitmask of pushbutton events. This will be a combination of
 *                  @ref PLATFORM_PB_ONBOARD_PRESS and/or @ref PLATFORM_PB_ONBOARD_RELEASE,
 *                  or 0 if no new events occurred.
 */
uint16_t platform_pb_get_event(void)
{
	uint16_t cache = pb_press_mask; // Store the current event mask in a temporary variable.
	
	pb_press_mask = 0; // Clear the global event mask to detect new events.
	return cache;      // Return the captured events.
}

//////////////////////////////////////////////////////////////////////////////
// Nested Vectored Interrupt Controller (NVIC) Configuration
//////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the Nested Vectored Interrupt Controller (NVIC).
 *
 * This function performs the following actions:
 * 1. Ensures all memory operations are complete using a Data Memory Barrier (__DMB).
 * 2. Enables global interrupts at the CPU level (__enable_irq).
 * 3. Sets the priority for the EIC_EXTINT[2] interrupt (pushbutton) to 3.
 * 4. Sets the priority for the SysTick interrupt to 3.
 * 5. Enables the EIC_EXTINT[2] interrupt in the NVIC.
 * 6. Enables the SysTick interrupt in the NVIC.
 *
 * @note This function should typically be called late in the platform initialization
 *       sequence, as interrupts will be globally enabled upon its completion.
 *       The NVIC itself is part of the ARM Cortex-M core and is always present;
 *       this function configures how it handles specific interrupt sources.
 */
static void NVIC_init(void)
{
	/*
	 * Unlike AHB/APB peripherals, the NVIC is part of the Arm v8-M
	 * architecture core proper. Hence, it is always enabled.
	 */
	__DMB(); // Data Memory Barrier: ensures all explicit memory accesses before this complete.
	__enable_irq(); // Enable global interrupts at the CPU core level.

	// Set priority for EIC_EXTINT_2 interrupt (IRQ number 5 for PIC32CM LS00). Lower value = higher priority.
	NVIC_SetPriority(EIC_EXTINT_2_IRQn, 3);
	// Set priority for SysTick interrupt (IRQ number -1).
	NVIC_SetPriority(SysTick_IRQn, 3);

	// Enable EIC_EXTINT_2 interrupt in the NVIC.
	NVIC_EnableIRQ(EIC_EXTINT_2_IRQn);
	// Enable SysTick interrupt in the NVIC.
	NVIC_EnableIRQ(SysTick_IRQn);
	return;
}

/////////////////////////////////////////////////////////////////////////////
// Platform Initialization and Main Loop Hook
/////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the entire platform hardware and software components.
 *
 * This function serves as the main entry point for platform initialization.
 * It orchestrates calls to various initialization routines in a specific order
 * to ensure correct setup of clocks, peripherals, and interrupts:
 * 1. `raise_perf_level()`: Configures system clocks and performance level.
 * 2. `EVSYS_init()`: Initializes the Event System.
 * 3. `EIC_init_early()`: Performs early setup of the External Interrupt Controller.
 * 4. `PB_init()`: Initializes pushbutton inputs.
 * 5. `GPO_init()`: Initializes General Purpose Outputs (LEDs).
 * 6. `platform_usart_init()`: Initializes the main USART communication interface.
 * 7. `gps_platform_usart_init()`: Initializes the USART interface for the GPS module.
 * 8. `EIC_init_late()`: Completes EIC setup by enabling it.
 * 9. `platform_systick_init()`: Initializes the SysTick timer for system timing.
 * 10. `NVIC_init()`: Configures and enables interrupts in the NVIC.
 *
 * @note This function should be called once at the very beginning of the application,
 *       before the main application loop starts.
 */
void platform_init(void)
{
	// Raise the power level and configure system clocks
	raise_perf_level();
	
	// Early initialization of peripherals that need setup before others or before being fully enabled
	EVSYS_init();
	EIC_init_early();
	
	// Regular initialization of core I/O and communication peripherals
	PB_init();
	GPO_init();
	platform_usart_init();
    gps_platform_usart_init();
	
	// Late initialization, typically enabling peripherals or interrupts after full configuration
	EIC_init_late();
	platform_systick_init();
	NVIC_init(); // This will enable global interrupts as one of its final steps.
	return;
}

/**
 * @brief Executes one iteration of platform-specific tasks for the main event loop.
 *
 * This function is intended to be called repeatedly within the application's
 * main infinite loop. It performs tasks that need regular servicing, such as:
 * - Retrieving the current high-resolution tick count using `platform_tick_hrcount()`.
 * - Calling tick handlers for the main USART and GPS USART modules, allowing them
 *   to process timeouts, manage buffers, or perform other time-sensitive operations.
 *
 * @note This function helps decouple periodic platform-level tasks from the main
 *       application logic, promoting modularity.
 */
void platform_do_loop_one(void)
{
    
	/*
	 * Some routines must be serviced as quickly as is practicable. Do so
	 * now.
	 */
	platform_timespec_t tick; // Structure to hold the current time/tick count.
	
	// Get the current high-resolution system tick count.
	platform_tick_hrcount(&tick);
    
	// Call the tick handler for the main USART peripheral, passing the current tick.
	platform_usart_tick_handler(&tick);
	// Call the tick handler for the GPS USART peripheral, passing the current tick.
	gps_platform_usart_tick_handler(&tick);
}