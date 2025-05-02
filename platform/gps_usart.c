/**
 * @file platform/usart.c
 * @brief Platform-support routines, USART component
 *
 * @author Alberto de Villa <alberto.de.villa@eee.upd.edu.ph>
 * @date   28 Oct 2024
 */

/*
 * PIC32CM5164LS00048 initial configuration:
 * -- Architecture: ARMv8 Cortex-M23
 * -- GCLK_GEN0: OSC16M @ 4 MHz, no additional prescaler
 * -- Main Clock: No additional prescaling (always uses GCLK_GEN0 as input)
 * -- Mode: Secure, NONSEC disabled
 * 
 * HW configuration for the corresponding Curiosity Nano+ Touch Evaluation
 * Board:
 * -- PB17: UART via debugger (RX, SERCOM1, PAD[1])
 */

// Common include for the XC32 compiler
#include <xc.h>
#include <stdbool.h>
#include <string.h>

#include "../platform.h"

// Functions "exported" by this file
void gps_platform_usart_init(void);
void gps_platform_usart_tick_handler(const platform_timespec_t *tick);

/////////////////////////////////////////////////////////////////////////////

/**
 * State variables for UART
 * 
 * NOTE: Since these are shared between application code and interrupt handlers
 *       (SysTick and SERCOM), these must be declared volatile.
 */
typedef struct ctx_usart_type {
	
	/// Pointer to the underlying register set
	sercom_usart_int_registers_t *regs;
	
	/// State variables for the transmitter
	struct {
		volatile platform_usart_tx_bufdesc_t *desc;
		volatile uint16_t nr_desc;
		
		// Current descriptor
		volatile const char *buf;
		volatile uint16_t    len;
	} tx;
	
	/// State variables for the receiver
	struct {
		/// Receive descriptor, held by the client
		volatile platform_usart_rx_async_desc_t * volatile desc;
		
		/// Tick since the last character was received
		volatile platform_timespec_t ts_idle;
		
		/// Index at which to place an incoming character
		volatile uint16_t idx;
				
	} rx;
	
	/// Configuration items
	struct {
		/// Idle timeout (reception only)
		platform_timespec_t ts_idle_timeout;
	} cfg;
	
} gps_ctx_usart_t;
static gps_ctx_usart_t gps_ctx_uart;

// Configure USART
void gps_platform_usart_init(void){
	/*
	 * For ease of typing, #define a macro corresponding to the SERCOM
	 * peripheral and its internally-clocked USART view.
	 * 
	 * To avoid namespace pollution, this macro is #undef'd at the end of
	 * this function.
	 */
#define UART_REGS (&(SERCOM1_REGS->USART_INT))
	
	/*
	 * Enable the APB clock for this peripheral
	 * 
	 * NOTE: The chip resets with it enabled; hence, commented-out.
	 * 
	 * WARNING: Incorrect MCLK settings can cause system lockup that can
	 *          only be rectified via a hardware reset/power-cycle.
	 */
	// MCLK_REGS->MCLK_APB???MASK |= (1 << ???);
	
	/*
	 * Enable the GCLK generator for this peripheral
	 * 
	 * NOTE: GEN2 (4 MHz) is used, as GEN0 (24 MHz) is too fast for our
	 *       use case.
	 */
	GCLK_REGS->GCLK_PCHCTRL[18] = 0x00000042;
	while ((GCLK_REGS->GCLK_PCHCTRL[18] & 0x00000040) == 0) asm("nop");
	
	// Initialize the peripheral's context structure
	memset(&gps_ctx_uart, 0, sizeof(gps_ctx_uart));
	gps_ctx_uart.regs = UART_REGS;
	
	/*
	 * This is the classic "SWRST" (software-triggered reset).
	 * 
	 * NOTE: Like the TC peripheral, SERCOM has differing views depending
	 *       on operating mode (USART_INT for UART mode). CTRLA is shared
	 *       across all modes, so set it first after reset.
	 */
	UART_REGS->SERCOM_CTRLA = (0x1 << 0);
	while((UART_REGS->SERCOM_SYNCBUSY & (0x1 << 0)) != 0) asm("nop");
	UART_REGS->SERCOM_CTRLA = (uint32_t)(0x1 << 2);
		
	/*
	 * Select further settings compatible with the 16550 UART:
	 * 
	 * - 16-bit oversampling, arithmetic mode (for noise immunity) A
	 * - LSB first A 
	 * - No parity A
	 * - Two stop bits B
	 * - 8-bit character size B
	 * - No break detection 
	 * 
	 * - Use PAD[0] for data transmission A
	 * - Use PAD[1] for data reception A
	 * 
	 * NOTE: If a control register is not used, comment it out.
	 */
	UART_REGS->SERCOM_CTRLA |= (0x0 << 13) | (0x1 << 30) | (0x0 << 24) | (0x1 << 20);
	UART_REGS->SERCOM_CTRLB |= (0x1 << 6) | (0x0 << 0);
	//UART_REGS->SERCOM_CTRLC |= ???;
	
	/*
	 * This value is determined from f_{GCLK} and f_{baud}, the latter
	 * being the actual target baudrate (here, 38400 bps).
	 */
	UART_REGS->SERCOM_BAUD = 0xF62B;
	
	/*
	 * Configure the IDLE timeout, which should be the length of 3
	 * USART characters.
	 * 
	 * NOTE: Each character is composed of 8 bits (must include parity
	 *       and stop bits); add one bit for margin purposes. In addition,
	 *       for UART one baud period corresponds to one bit.
	 */
	gps_ctx_uart.cfg.ts_idle_timeout.nr_sec  = 0;
	gps_ctx_uart.cfg.ts_idle_timeout.nr_nsec = 781250;
	
	/*
	 * Third-to-the-last setup:
	 * 
	 * - Enable receiver and transmitter
	 * - Clear the FIFOs (even though they're disabled)
	 */
    
	UART_REGS->SERCOM_CTRLB |= (0x1 << 17) | (0x1 << 16) | (0x3 << 22);
	while ((UART_REGS->SERCOM_SYNCBUSY & (0x1 << 2)) != 0) asm("nop");
    
	/*
	 * Second-to-last: Configure the physical pins.
	 * 
	 * NOTE: Consult both the chip and board datasheets to determine the
	 *       correct port pins to use.
	 */
    PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = (1 << 17);
	PORT_SEC_REGS->GROUP[0].PORT_PINCFG[17] = 0x3;
	PORT_SEC_REGS->GROUP[0].PORT_PMUX[17 >> 1] = 0x20;
    
    
    // Last: enable the peripheral, after resetting the state machine
	UART_REGS->SERCOM_CTRLA |= (0x1 << 1);
	while ((UART_REGS->SERCOM_SYNCBUSY & (0x1 << 1)) != 0) asm("nop");
	return;

#undef UART_REGS
}

// Helper abort routine for USART reception
static void gps_usart_rx_abort_helper(gps_ctx_usart_t *ctx)
{
	if (ctx->rx.desc != NULL) {
		ctx->rx.desc->compl_type = PLATFORM_USART_RX_COMPL_DATA;
		ctx->rx.desc->compl_info.data_len = ctx->rx.idx;
		ctx->rx.desc = NULL;
	}
	ctx->rx.ts_idle.nr_sec  = 0;
	ctx->rx.ts_idle.nr_nsec = 0;
	ctx->rx.idx = 0;
	return;
}

// Tick handler for the USART
static void gps_usart_tick_handler_common(
	gps_ctx_usart_t *ctx, const platform_timespec_t *tick)
{
	uint16_t status = 0x0000;
	uint8_t  data   = 0x00;
	platform_timespec_t ts_delta;
	
	// RX handling
	if ((ctx->regs->SERCOM_INTFLAG & (1 << 2)) != 0) {
		/*
		 * There are unread data
		 * 
		 * To enable readout of error conditions, STATUS must be read
		 * before reading DATA.
		 * 
		 * NOTE: Piggyback on Bit 15, as it is undefined for this
		 *       platform.
		 */
		status = ctx->regs->SERCOM_STATUS | 0x8000;
		data   = (uint8_t)(ctx->regs->SERCOM_DATA);
	}
	do {
		if (ctx->rx.desc == NULL) {
			// Nowhere to store any read data
			break;
		}

		if ((status & 0x8003) == 0x8000) {
			// No errors detected
			ctx->rx.desc->buf[ctx->rx.idx++] = data;
			ctx->rx.ts_idle = *tick;
		}
		ctx->regs->SERCOM_STATUS |= (status & 0x00F7);

		// Some housekeeping
		if (ctx->rx.idx >= ctx->rx.desc->max_len) {
			// Buffer completely filled
			gps_usart_rx_abort_helper(ctx);
			break;
		} else if (ctx->rx.idx > 0) {
			platform_tick_delta(&ts_delta, tick, &ctx->rx.ts_idle);
			if (platform_timespec_compare(&ts_delta, &ctx->cfg.ts_idle_timeout) >= 0) {
				// IDLE timeout
				gps_usart_rx_abort_helper(ctx);
				break;
			}
		}
	} while (0);
	
	// Done
	return;
}
void gps_platform_usart_tick_handler(const platform_timespec_t *tick)
{
	gps_usart_tick_handler_common(&gps_ctx_uart, tick);
}

/// Maximum number of bytes that may be sent (or received) in one transaction
#define NR_USART_CHARS_MAX (65528)

/// Maximum number of fragments for USART TX
#define NR_USART_TX_FRAG_MAX (32)

// Begin a receive transaction
static bool gps_usart_rx_busy(gps_ctx_usart_t *ctx)
{
	return (ctx->rx.desc) != NULL;
}
static bool gps_usart_rx_async(gps_ctx_usart_t *ctx, platform_usart_rx_async_desc_t *desc)
{
	// Check some items first
	if (!desc|| !desc->buf || desc->max_len == 0 || desc->max_len > NR_USART_CHARS_MAX)
		// Invalid descriptor
		return false;
	
	if ((ctx->rx.desc) != NULL)
		// Don't clobber an existing buffer
		return false;
	
	desc->compl_type = PLATFORM_USART_RX_COMPL_NONE;
	desc->compl_info.data_len = 0;
	ctx->rx.idx = 0;
	platform_tick_hrcount(&ctx->rx.ts_idle);
	ctx->rx.desc = desc;
	return true;
}

// API-visible items
bool gps_platform_usart_cdc_rx_async(platform_usart_rx_async_desc_t *desc)
{
	return gps_usart_rx_async(&gps_ctx_uart, desc);
}
bool gps_platform_usart_cdc_rx_busy(void)
{
	return gps_usart_rx_busy(&gps_ctx_uart);
}
void gps_platform_usart_cdc_rx_abort(void)
{
	gps_usart_rx_abort_helper(&gps_ctx_uart);
}
