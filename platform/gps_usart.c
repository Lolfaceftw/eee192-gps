/**
 * @file      platform/gps_usart.c
 * @brief     Platform-support routines for the GPS USART (SERCOM1) component.
 *
 * This file implements the USART (Universal Synchronous/Asynchronous Receiver/Transmitter)
 * driver specifically for communication with a GPS module, utilizing SERCOM1.
 * It handles initialization of the SERCOM peripheral in USART mode,
 * asynchronous data reception, and provides a tick handler for managing
 * reception timeouts.
 *
 * @author    Alberto de Villa <alberto.de.villa@eee.upd.edu.ph> (Original Structure)
 * @author    Christian Klein C. Ramos (Docstrings, Comments, Adherence to Project Guidelines)
 * @date      07 May 2025
 *
 * @note      This file was originally named `usart.c` and has been adapted or
 *            is intended for GPS communication. The naming convention `gps_`
 *            has been applied to public functions and the context structure.
 */

/*
 * PIC32CM5164LS00048 initial configuration:
 * -- Architecture: ARMv8 Cortex-M23
 * -- GCLK_GEN0: OSC16M @ 4 MHz, no additional prescaler
 * -- Main Clock: No additional prescaling (always uses GCLK_GEN0 as input)
 * -- Mode: Secure, NONSEC disabled
 * 
 * HW configuration for the corresponding Curiosity Nano+ Touch Evaluation
 * Board (relevant to this SERCOM1 configuration):
 * -- PB17: UART via debugger (RX, SERCOM1, PAD[1])
 *    (Note: The code configures PAD[0] for TX and PAD[1] for RX, which implies
 *     PB16 might be TX if using standard SERCOM1 ALT pinout for these pads.)
 */

// Common include for the XC32 compiler, which includes device-specific headers
#include <xc.h>
#include <stdbool.h> // For boolean type (true, false)
#include <string.h>  // For memset

#include "../platform.h" // For platform_timespec_t, platform_usart_tx_bufdesc_t, platform_usart_rx_async_desc_t, etc.

// Functions "exported" by this file (as per original comment, though they are static or part of the API)
// Public API functions are declared in platform.h and defined at the end of this file.
// Static functions are internal to this module.

/**
 * @brief Initializes the USART peripheral (SERCOM1) for GPS communication.
 *
 * This function performs the necessary steps to configure SERCOM1 for USART operation:
 * 1. Enables the peripheral clock (GCLK) for SERCOM1, using GCLK_GEN2 (4 MHz).
 * 2. Initializes the context structure `gps_ctx_uart` for managing SERCOM1 state.
 * 3. Resets the SERCOM1 peripheral using a software reset.
 * 4. Configures USART operational parameters:
 *    - Mode: USART with internal clock.
 *    - Sampling: 16x oversampling, arithmetic mode.
 *    - Data order: LSB first.
 *    - Parity: None.
 *    - Stop bits: Two. (Original code sets CTRLB.SBMODE to 0x1 for two stop bits)
 *    - Character size: 8-bit.
 *    - RX Pad: PAD[1]. TX Pad: PAD[0].
 * 5. Sets the baud rate register for a target baud rate (e.g., 38400 bps from 4 MHz GCLK).
 * 6. Configures an idle timeout for reception (3 character times).
 * 7. Enables the receiver and transmitter.
 * 8. Configures the physical I/O pins (PB17 for RX - SERCOM1/PAD[1]) for SERCOM1 functionality.
 *    (Note: TX pin PB16 for SERCOM1/PAD[0] would also need configuration if TX was used by this module).
 * 9. Enables the SERCOM1 peripheral.
 */
void gps_platform_usart_init(void); // Public function, definition below

/**
 * @brief Tick handler for the GPS USART (SERCOM1).
 *
 * This function is intended to be called periodically (e.g., from a system tick interrupt
 * or main loop) to manage time-sensitive aspects of USART operation, primarily for
 * handling reception timeouts. It calls `gps_usart_tick_handler_common`.
 *
 * @param[in] tick Pointer to a `platform_timespec_t` structure representing the current system time/tick.
 */
void gps_platform_usart_tick_handler(const platform_timespec_t *tick); // Public function, definition below

/////////////////////////////////////////////////////////////////////////////

/**
 * @brief State variables for a single USART (SERCOM) peripheral instance.
 *
 * This structure holds all the necessary state information for managing
 * asynchronous transmission and reception on a SERCOM peripheral configured
 * in USART mode. This includes pointers to hardware registers, transmit/receive
 * buffer descriptors, current transfer progress, and configuration settings like
 * idle timeouts.
 *
 * @note Since these members can be accessed by both application code and
 *       interrupt handlers (implicitly, via the tick handler which might be
 *       called from an ISR context, or if SERCOM interrupts were used),
 *       relevant members are declared `volatile`.
 */
typedef struct ctx_usart_type {
	
	/// Pointer to the underlying SERCOM USART register set (e.g., SERCOM1_REGS->USART_INT).
	sercom_usart_int_registers_t *regs;
	
	/// State variables for the transmitter.
	struct {
		/** @brief Pointer to an array of transmit buffer descriptors. (Not used in this GPS specific file) */
		volatile platform_usart_tx_bufdesc_t *desc;
		/** @brief Number of descriptors in the transmit array. (Not used) */
		volatile uint16_t nr_desc;
		
		// Current descriptor being transmitted (Not used)
		/** @brief Pointer to the current transmit buffer. (Not used) */
		volatile const char *buf;
		/** @brief Number of bytes remaining in the current transmit buffer. (Not used) */
		volatile uint16_t    len;
	} tx;
	
	/// State variables for the receiver.
	struct {
		/** @brief Pointer to the active asynchronous receive descriptor, provided by the client. */
		volatile platform_usart_rx_async_desc_t * volatile desc;
		
		/** @brief Timestamp (system tick) of when the last character was received. Used for idle detection. */
		volatile platform_timespec_t ts_idle;
		
		/** @brief Current index within the receive buffer (`desc->buf`) where the next incoming character will be placed. */
		volatile uint16_t idx;
				
	} rx;
	
	/// Configuration items for this USART instance.
	struct {
		/** @brief Idle timeout duration for reception. If no character is received
		 *         within this period, an ongoing reception may be considered complete.
         */
		platform_timespec_t ts_idle_timeout;
	} cfg;
	
} gps_ctx_usart_t;

/** @brief Static instance of the USART context structure for GPS communication (SERCOM1). */
static gps_ctx_usart_t gps_ctx_uart;

// Configure USART (SERCOM1 for GPS)
/**
 * @brief Initializes the USART peripheral (SERCOM1) for GPS communication.
 *
 * (Detailed Doxygen for this function is provided with its prototype declaration earlier.)
 */
void gps_platform_usart_init(void){
	/*
	 * For ease of typing, #define a macro corresponding to the SERCOM
	 * peripheral and its internally-clocked USART view.
	 * 
	 * To avoid namespace pollution, this macro is #undef'd at the end of
	 * this function.
	 */
#define UART_REGS (&(SERCOM1_REGS->USART_INT)) // Define UART_REGS to point to SERCOM1's USART interface
	
	/*
	 * Enable the APB clock for this peripheral (SERCOM1)
	 * 
	 * NOTE: The chip resets with it enabled; hence, commented-out.
	 *       SERCOM1 is on APBC bus. MCLK_APBCMASK bit for SERCOM1 is ID_SERCOM1 - 64 = 2.
	 * 
	 * WARNING: Incorrect MCLK settings can cause system lockup that can
	 *          only be rectified via a hardware reset/power-cycle.
	 */
	// MCLK_REGS->MCLK_APBCMASK |= (1 << 2); // MCLK_APBCMASK_SERCOM1_Pos = 2
	
	/*
	 * Enable the GCLK generator for this peripheral (SERCOM1)
	 * 
	 * NOTE: GEN2 (4 MHz) is used, as GEN0 (24 MHz) is too fast for our
	 *       use case. SERCOM1 GCLK ID is 18 (SERCOM1_GCLK_ID_CORE).
	 */
	// Configure GCLK_PCHCTRL[18] for SERCOM1: Enable channel (CHEN, bit 6), Source GCLK_GEN2 (GEN bits 3:0 = 0x02).
	// Value = (1 << 6) | 0x02 = 0x42.
	GCLK_REGS->GCLK_PCHCTRL[18] = 0x00000042;
	// Wait for GCLK_PCHCTRL[18].CHEN (bit 6) to be set, indicating channel is enabled.
	// Original check was `(GCLK_REGS->GCLK_PCHCTRL[18] & 0x00000040) == 0`. 0x40 is (1<<6).
	while ((GCLK_REGS->GCLK_PCHCTRL[18] & 0x00000040) == 0) asm("nop");
	
	// Initialize the peripheral's context structure to all zeros.
	memset(&gps_ctx_uart, 0, sizeof(gps_ctx_uart));
	// Store a pointer to the SERCOM1 USART registers in the context.
	gps_ctx_uart.regs = UART_REGS;
	
	/*
	 * This is the classic "SWRST" (software-triggered reset).
	 * 
	 * NOTE: Like the TC peripheral, SERCOM has differing views depending
	 *       on operating mode (USART_INT for UART mode). CTRLA is shared
	 *       across all modes, so set it first after reset.
	 */
	// Perform software reset: Set SWRST bit (bit 0) in SERCOM_CTRLA.
	UART_REGS->SERCOM_CTRLA = (0x1 << 0);
	// Wait for SWRST synchronization to complete (SYNCBUSY bit 0 for SWRST).
	while((UART_REGS->SERCOM_SYNCBUSY & (0x1 << 0)) != 0) asm("nop");

    // Configure SERCOM_CTRLA for USART mode with internal clock.
    // Set MODE to USART with internal clock (bits 4:2 = 0x1). Value (0x1 << 2).
	UART_REGS->SERCOM_CTRLA = (uint32_t)(0x1 << 2);
		
	/*
	 * Select further settings compatible with the 16550 UART:
	 * 
	 * - 16-bit oversampling, arithmetic mode (for noise immunity) A -> SAMPR=0 in CTRLA
	 * - LSB first A -> DORD=1 (default) in CTRLA
	 * - No parity A -> FORM bits 3:1 = 0x0 (USART frame with no parity) in CTRLA
	 * - Two stop bits B -> SBMODE=1 (2 stop bits) in CTRLB
	 * - 8-bit character size B -> CHSIZE bits 2:0 = 0x0 (8 bits) in CTRLB
	 * - No break detection 
	 * 
	 * - Use PAD[0] for data transmission A -> TXPO bits 1:0 = 0x0 (SERCOM PAD[0] for TXD) in CTRLA
	 * - Use PAD[1] for data reception A -> RXPO bits 1:0 = 0x1 (SERCOM PAD[1] for RXD) in CTRLA
	 * 
	 * NOTE: If a control register is not used, comment it out.
	 */
    // Configure SERCOM_CTRLA:
    // SAMPR (Oversampling, bits 15:13) = 0x0 (16x arithmetic).
    // DORD (Data Order, bit 30) = 1 (LSB first - this is default, so 0x1<<30 is not strictly needed if register is reset).
    // FORM (Frame Format, bits 27:24) = 0x0 (USART frame, no parity).
    // RXPO (Receive Data Pinout, bits 21:20) = 0x1 (PAD[1]).
    // TXPO (Transmit Data Pinout, bits 1:0 of CTRLA in some views, but for USART_INT it's bits 17:16 of CTRLA)
    // The original code `(0x0 << 13) | (0x1 << 30) | (0x0 << 24) | (0x1 << 20)` corresponds to:
    // SAMPR=0, DORD=1, FORM=0, RXPO=1. TXPO is not set here.
	UART_REGS->SERCOM_CTRLA |= (0x0 << 13) | (0x1 << 30) | (0x0 << 24) | (0x1 << 20);

    // Configure SERCOM_CTRLB:
    // SBMODE (Stop Bit Mode, bit 6) = 1 (2 stop bits).
    // CHSIZE (Character Size, bits 2:0) = 0x0 (8 bits).
	UART_REGS->SERCOM_CTRLB |= (0x1 << 6) | (0x0 << 0);
	//UART_REGS->SERCOM_CTRLC |= ???; // CTRLC not used in this configuration.
	
	/*
	 * This value is determined from f_{GCLK} and f_{baud}, the latter
	 * being the actual target baudrate (here, 38400 bps).
     * Baud value = 65536 * (1 - 16 * f_baud / f_gclk)
     * For f_gclk = 4MHz, f_baud = 38400:
     * BAUD = 65536 * (1 - 16 * 38400 / 4000000) = 65536 * (1 - 0.1536) = 65536 * 0.8464 = 55471.59
     * Decimal 55471 is 0xD8AF.
     * The original value 0xF62B (decimal 63019) implies a different calculation or target.
     * If using fractional baud generation (CTRLA.SAMPA=1), BAUD = f_gclk / f_baud.
     * If using arithmetic (CTRLA.SAMPA=0), BAUD = f_gclk / (16 * f_baud) - 1 (approximately for BAUD.FP part, this is simplified).
     * The value 0xF62B seems to be for a different clock or baud rate, or a specific calculation method.
     * We will use the original value as per instructions.
	 */
	UART_REGS->SERCOM_BAUD = 0xF62B; // Set Baud register.
	
	/*
	 * Configure the IDLE timeout, which should be the length of 3
	 * USART characters.
	 * 
	 * NOTE: Each character is composed of 8 bits (must include parity
	 *       and stop bits); add one bit for margin purposes. In addition,
	 *       for UART one baud period corresponds to one bit.
     * Assuming 1 start bit, 8 data bits, 2 stop bits = 11 bits per character.
     * 3 characters = 33 bits.
     * Time for 1 bit = 1 / 38400 bps = 26.0416 microseconds.
     * Time for 33 bits = 33 * 26.0416 us = 859.375 us.
     * The original value 781250 ns = 781.25 us is close to this (30 bits).
	 */
	gps_ctx_uart.cfg.ts_idle_timeout.nr_sec  = 0;        // Seconds part of idle timeout.
	gps_ctx_uart.cfg.ts_idle_timeout.nr_nsec = 781250;   // Nanoseconds part of idle timeout.
	
	/*
	 * Third-to-the-last setup:
	 * 
	 * - Enable receiver and transmitter
	 * - Clear the FIFOs (even though they're disabled)
	 */
    // Enable Receiver (RXEN, bit 17) and Transmitter (TXEN, bit 16) in SERCOM_CTRLB.
    // LINCMD (bits 23:22) = 0x3 (No action, or specific command if used).
    // Original code uses `(0x3 << 22)` which might be for a specific LIN command or a typo if FIFOs are not used.
    // If FIFOs are disabled, clearing them has no effect. Let's assume it's for general robustness or future use.
	UART_REGS->SERCOM_CTRLB |= (0x1 << 17) | (0x1 << 16) | (0x3 << 22);
    // Wait for CTRLB synchronization (SYNCBUSY bit 2 for CTRLB).
	while ((UART_REGS->SERCOM_SYNCBUSY & (0x1 << 2)) != 0) asm("nop");
    
	/*
	 * Second-to-last: Configure the physical pins.
	 * 
	 * NOTE: Consult both the chip and board datasheets to determine the
	 *       correct port pins to use.
     * For SERCOM1:
     *  - RX on PAD[1] -> PB17 (as per file header comment)
     *  - TX on PAD[0] -> PB16 (typical for SERCOM1 ALT)
     * Pin configuration for PB17 (RX):
     *  - Direction: Input (DIRCLR for PB17).
     *  - PINCFG: Enable peripheral MUX (PMUXEN=1, bit 0), Enable input buffer (INEN=1, bit 1). Value 0x3.
     *  - PMUX: Select SERCOM1 Alternate function (Function D = 0x3).
     *    PB17 is an odd pin, uses PMUXO field (bits 7:4) in PORT_PMUX[17>>1] = PORT_PMUX[8].
     *    PMUXO = 0x3. So, (0x3 << 4) = 0x30.
     * The original code uses 0x20 for PMUX, which is Function C. This needs to match the datasheet for SERCOM1_ALT.
     * Assuming Function C (0x2) is correct for SERCOM1_ALT on PB17.
	 */
    // Configure PB17 (SERCOM1 PAD[1] - RX)
    PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = (1 << 17);      // Set PB17 direction to input.
	PORT_SEC_REGS->GROUP[0].PORT_PINCFG[17] = 0x3;        // Enable PMUX and INEN for PB17.
	PORT_SEC_REGS->GROUP[0].PORT_PMUX[17 >> 1] = 0x20;    // Set PMUXO for PB17 to Function C (0x2).
    // Note: TX pin (e.g., PB16 for PAD[0]) configuration would be needed if transmitting.
    
    
    // Last: enable the peripheral, after resetting the state machine
    // Enable SERCOM peripheral: Set ENABLE bit (bit 1) in SERCOM_CTRLA.
	UART_REGS->SERCOM_CTRLA |= (0x1 << 1);
    // Wait for ENABLE synchronization (SYNCBUSY bit 1 for ENABLE).
	while ((UART_REGS->SERCOM_SYNCBUSY & (0x1 << 1)) != 0) asm("nop");
	return;

#undef UART_REGS // Undefine the local macro
}

/**
 * @brief Helper function to abort an ongoing USART reception.
 *
 * This function is called when a reception needs to be terminated, either due to
 * the buffer being full, an idle timeout, or an explicit abort request.
 * If an active receive descriptor (`ctx->rx.desc`) exists, it updates the
 * descriptor's completion status (`compl_type` to `PLATFORM_USART_RX_COMPL_DATA`)
 * and the number of bytes received (`compl_info.data_len`). It then clears
 * the active descriptor pointer, effectively making the receiver idle.
 * It also resets the idle timestamp and buffer index.
 *
 * @param[in,out] ctx Pointer to the `gps_ctx_usart_t` structure for the USART instance.
 *                    Modifies `ctx->rx.desc`, `ctx->rx.ts_idle`, and `ctx->rx.idx`.
 */
static void gps_usart_rx_abort_helper(gps_ctx_usart_t *ctx)
{
    // Check if there is an active receive descriptor.
	if (ctx->rx.desc != NULL) {
        // Mark the reception as complete due to data received (or timeout/full buffer).
		ctx->rx.desc->compl_type = PLATFORM_USART_RX_COMPL_DATA;
        // Store the number of bytes actually received in the descriptor.
		ctx->rx.desc->compl_info.data_len = ctx->rx.idx;
        // Clear the pointer to the active descriptor, making the receiver available.
		ctx->rx.desc = NULL;
	}
    // Reset the idle timestamp.
	ctx->rx.ts_idle.nr_sec  = 0;
	ctx->rx.ts_idle.nr_nsec = 0;
    // Reset the receive buffer index.
	ctx->rx.idx = 0;
	return;
}

/**
 * @brief Common tick handler logic for USART reception.
 *
 * This function is called by `gps_platform_usart_tick_handler`. It performs the following:
 * 1. Checks the USART's interrupt flag register for received data (RXC flag).
 * 2. If data is available (RXC is set):
 *    a. Reads the STATUS register to get error flags (and clear them by writing back).
 *    b. Reads the DATA register to get the received byte.
 *    c. If an active receive descriptor (`ctx->rx.desc`) exists and no errors were detected:
 *       i. Stores the received byte in the client's buffer (`ctx->rx.desc->buf`).
 *       ii. Increments the buffer index (`ctx->rx.idx`).
 *       iii. Updates the idle timestamp (`ctx->rx.ts_idle`) to the current tick.
 * 3. Checks for receive buffer full condition:
 *    a. If `ctx->rx.idx` reaches `ctx->rx.desc->max_len`, calls `gps_usart_rx_abort_helper`
 *       to complete the reception.
 * 4. Checks for receive idle timeout:
 *    a. If `ctx->rx.idx` > 0 (meaning some data has been received), calculates the time
 *       delta between the current tick and `ctx->rx.ts_idle`.
 *    b. If this delta is greater than or equal to `ctx->cfg.ts_idle_timeout`, calls
 *       `gps_usart_rx_abort_helper` to complete the reception due to timeout.
 *
 * @param[in,out] ctx  Pointer to the `gps_ctx_usart_t` structure for the USART instance.
 * @param[in]     tick Pointer to a `platform_timespec_t` structure representing the current system time.
 */
static void gps_usart_tick_handler_common(
	gps_ctx_usart_t *ctx, const platform_timespec_t *tick)
{
	uint16_t status = 0x0000; // To store SERCOM_STATUS register value.
	uint8_t  data   = 0x00;   // To store received data byte.
	platform_timespec_t ts_delta; // To store time difference for idle timeout calculation.
	
	// RX handling: Check if Receive Complete Interrupt Flag (RXC, bit 2 of SERCOM_INTFLAG) is set.
	if ((ctx->regs->SERCOM_INTFLAG & (1 << 2)) != 0) {
		/*
		 * There are unread data.
		 * 
		 * To enable readout of error conditions, STATUS must be read
		 * before reading DATA.
		 * 
		 * NOTE: Piggyback on Bit 15, as it is undefined for this
		 *       platform. (Original comment, implies status bit 15 is used as a flag
         *       to indicate data was read from hardware in this tick cycle).
		 */
        // Read STATUS register. This also clears error flags if they were set.
		status = ctx->regs->SERCOM_STATUS;
        // Set a custom flag (bit 15) in the local status variable to indicate data was read from hardware.
        status |= 0x8000; 
        // Read the received data from DATA register.
		data   = (uint8_t)(ctx->regs->SERCOM_DATA);
	}

    // This do-while(0) loop is a common C idiom for creating a single-pass block
    // that can be exited early using 'break'.
	do {
        // If there is no active receive descriptor, we have nowhere to store received data.
		if (ctx->rx.desc == NULL) {
			break; // Exit processing for this tick.
		}

        // Check if data was read from hardware in this cycle (status bit 15 set)
        // AND if there are no USART errors (status bits 1:0 for Parity/Frame error, bit 2 for Buffer Overflow).
        // Original check `(status & 0x8003) == 0x8000` means:
        // - Bit 15 must be set (our flag indicating data was read from HW).
        // - Bits 1 and 0 (PERR, FERR) must be clear. Bit 2 (BUFOVF) is not checked by this mask.
        // A more complete error check might be `(status & (PERR_Msk | FERR_Msk | BUFOVF_Msk)) == 0`.
        // We use the original check.
		if ((status & 0x8003) == 0x8000) { // Check our "data read" flag and PERR/FERR.
			// No errors detected (or errors are ignored by this specific check).
            // Store the received byte into the client's buffer at the current index.
			ctx->rx.desc->buf[ctx->rx.idx++] = data;
            // Update the timestamp of the last received character to the current tick.
			ctx->rx.ts_idle = *tick;
		}
        // Clear any error flags in the hardware STATUS register by writing them back.
        // Mask 0x00F7 clears bits 3 (ISF - Inconsistent Sync Field) and 7 (COLL - Collision),
        // while preserving other status bits that might be write-to-clear or have other meanings.
        // This seems to be an attempt to clear specific error flags that were read.
        // Standard way to clear error flags is to read STATUS then DATA, or write 1s to INTFLAG bits.
        // Writing to STATUS directly is less common but might be device-specific for clearing.
		ctx->regs->SERCOM_STATUS |= (status & 0x00F7); // This attempts to clear some status bits.

		// Some housekeeping for the receive buffer and idle timeout.
        // Check if the receive buffer is completely filled.
		if (ctx->rx.idx >= ctx->rx.desc->max_len) {
			// Buffer is full. Abort/complete the reception.
			gps_usart_rx_abort_helper(ctx);
			break; // Exit processing for this tick.
		} else if (ctx->rx.idx > 0) { // If some data has been received (buffer is not empty).
            // Calculate the time elapsed since the last character was received.
			platform_tick_delta(&ts_delta, tick, &ctx->rx.ts_idle);
            // Compare the elapsed idle time with the configured idle timeout.
			if (platform_timespec_compare(&ts_delta, &ctx->cfg.ts_idle_timeout) >= 0) {
				// IDLE timeout has occurred. Abort/complete the reception.
				gps_usart_rx_abort_helper(ctx);
				break; // Exit processing for this tick.
			}
		}
	} while (0); // End of single-pass block.
	
	// Done with USART tick handling for this call.
	return;
}

/**
 * @brief Public tick handler for the GPS USART (SERCOM1).
 *
 * This function is the externally visible tick handler. It simply calls the
 * common internal tick handler logic (`gps_usart_tick_handler_common`) with the
 * context for the GPS UART (`gps_ctx_uart`) and the provided current system tick.
 *
 * @param[in] tick Pointer to a `platform_timespec_t` structure representing the current system time.
 */
void gps_platform_usart_tick_handler(const platform_timespec_t *tick)
{
    // Call the common handler with the specific context for the GPS UART.
	gps_usart_tick_handler_common(&gps_ctx_uart, tick);
}

/// Maximum number of bytes that may be sent (or received) in one transaction.
// This is a large value, likely related to theoretical limits or DMA capabilities,
// not typically used for single small buffer transfers.
#define NR_USART_CHARS_MAX (65528)

/// Maximum number of fragments for USART TX. (Not used in this GPS RX-focused file).
#define NR_USART_TX_FRAG_MAX (32)

/**
 * @brief Checks if a USART receive operation is currently busy (active).
 *
 * @param[in] ctx Pointer to the `gps_ctx_usart_t` structure for the USART instance.
 * @return `true` if there is an active receive descriptor (`ctx->rx.desc` is not NULL),
 *         meaning a reception is in progress or has been set up.
 * @return `false` if the receiver is idle (no active descriptor).
 */
static bool gps_usart_rx_busy(gps_ctx_usart_t *ctx)
{
    // The receiver is busy if there's an active receive descriptor.
	return (ctx->rx.desc) != NULL;
}

/**
 * @brief Initiates an asynchronous USART receive operation.
 *
 * Sets up the USART context to receive data into the buffer specified by the
 * provided `desc` (platform_usart_rx_async_desc_t).
 *
 * @param[in,out] ctx  Pointer to the `gps_ctx_usart_t` structure for the USART instance.
 * @param[in,out] desc Pointer to a `platform_usart_rx_async_desc_t` structure
 *                     provided by the caller. This structure specifies the buffer,
 *                     its maximum length, and will be updated with completion status
 *                     and received data length.
 * @return `true` if the asynchronous receive operation was successfully initiated.
 * @return `false` if the parameters in `desc` are invalid (e.g., NULL buffer, zero length,
 *                 length too large) or if another receive operation is already in progress.
 */
static bool gps_usart_rx_async(gps_ctx_usart_t *ctx, platform_usart_rx_async_desc_t *desc)
{
	// Check some items first: validate the provided descriptor.
	if (!desc                       // Descriptor pointer must not be NULL.
     || !desc->buf                  // Buffer pointer in descriptor must not be NULL.
     || desc->max_len == 0          // Maximum buffer length must be greater than 0.
     || desc->max_len > NR_USART_CHARS_MAX) // Max length must not exceed defined limit.
		// Invalid descriptor.
		return false;
	
    // Check if another receive operation is already in progress.
	if ((ctx->rx.desc) != NULL)
		// Don't clobber an existing buffer/operation.
		return false;
	
    // Initialize the descriptor for the new reception.
	desc->compl_type = PLATFORM_USART_RX_COMPL_NONE; // No completion event yet.
	desc->compl_info.data_len = 0;                   // No data received yet.
    
    // Reset the context's receive buffer index.
	ctx->rx.idx = 0;
    // Record the current time as the start of the idle period (or start of reception).
	platform_tick_hrcount(&ctx->rx.ts_idle);
    // Store the pointer to the client's descriptor, making this the active reception.
	ctx->rx.desc = desc;
	return true; // Reception successfully initiated.
}

// API-visible items (public functions)

/**
 * @brief Initiates an asynchronous receive operation on the GPS USART (SERCOM1).
 *
 * This is the public API function that wraps `gps_usart_rx_async` for the
 * GPS UART context.
 *
 * @param[in,out] desc Pointer to a `platform_usart_rx_async_desc_t` structure
 *                     for the receive operation.
 * @return `true` if the receive operation was successfully initiated, `false` otherwise.
 */
bool gps_platform_usart_cdc_rx_async(platform_usart_rx_async_desc_t *desc)
{
    // Call the internal async receive function with the GPS UART context.
	return gps_usart_rx_async(&gps_ctx_uart, desc);
}

/**
 * @brief Checks if the GPS USART (SERCOM1) receiver is busy.
 *
 * This is the public API function that wraps `gps_usart_rx_busy` for the
 * GPS UART context.
 *
 * @return `true` if a receive operation is active, `false` otherwise.
 */
bool gps_platform_usart_cdc_rx_busy(void)
{
    // Call the internal busy check function with the GPS UART context.
	return gps_usart_rx_busy(&gps_ctx_uart);
}

/**
 * @brief Aborts an ongoing asynchronous receive operation on the GPS USART (SERCOM1).
 *
 * This is the public API function that wraps `gps_usart_rx_abort_helper` for the
 * GPS UART context. It immediately terminates any active reception.
 */
void gps_platform_usart_cdc_rx_abort(void)
{
    // Call the internal abort helper function with the GPS UART context.
	gps_usart_rx_abort_helper(&gps_ctx_uart);
}