#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>
#include <stdint.h>    // For uint16_t etc.
#include "platform.h"  // For platform_usart_tx_bufdesc_t, platform_usart_rx_async_desc_t

// --- Application Constants (Relevant to prog_state_t structure) ---
#define TX_BUFFER_SIZE_APP        128   // Size of the primary transmit buffer
#define RX_BUFFER_CDC_SIZE_APP    16    // Size of the CDC receive buffer
#define RX_BUFFER_GPS_SIZE_APP    64    // Size of the GPS UART receive buffer
#define GPS_ASSEMBLY_BUF_SIZE_APP 256   // Size of the GPS assembly buffer
#define MAX_GPGLL_STORE_LEN_APP   128   // Max length for storing a GPGLL sentence (content only, no CRLF)
                                        // (Used for the static buffer in main.c's prog_loop_one)

// --- Program State Flags ---
// These flags are part of prog_state_t and control the application's main logic flow.
#define PROG_FLAG_BANNER_PENDING        0x0001U // Indicates the banner message needs to be transmitted.
#define PROG_FLAG_GPS_UPDATE_PENDING    0x0002U // Indicates complete NMEA sentence(s) are in the GPS assembly buffer.
#define PROG_FLAG_PARSED_GPGLL_PENDING  0x0004U // Indicates a GPGLL sentence has been stored and awaits parsing/transmission.
#define PROG_FLAG_TX_BUFFER_BUSY        0x8000U // Indicates the main tx_buf (or banner) is populated and awaiting USART transmission completion.

/**
 * @brief Structure defining the main program state.
 *
 * Holds all runtime variables for the application, including flags,
 * transmit/receive buffers, and descriptors for asynchronous operations.
 * This structure is passed to various modules that operate on the application's state.
 */
typedef struct prog_state_type
{
	uint16_t flags;                             // Bitmask of PROG_FLAG_* values.
    bool banner_has_been_displayed_this_session; // Tracks if the banner was successfully queued for display at least once.

	// USART Transmit members
	platform_usart_tx_bufdesc_t tx_desc;        // Descriptor for asynchronous USART transmissions.
	char tx_buf[TX_BUFFER_SIZE_APP];            // Primary buffer for formatted output messages, shared with UI module.
	
	// USART Receive members (CDC for console/PC interaction)
	platform_usart_rx_async_desc_t cdc_rx_desc; // Descriptor for asynchronous CDC reception.
	char cdc_rx_buf[RX_BUFFER_CDC_SIZE_APP];    // Buffer for data received via CDC.
    
    // USART Receive members (GPS module)
    platform_usart_rx_async_desc_t gps_rx_desc; // Descriptor for asynchronous GPS UART reception.
    char gps_rx_buf[RX_BUFFER_GPS_SIZE_APP];    // Buffer for raw data chunks from the GPS module.
    
    // GPS NMEA Sentence Assembly
    char gps_assembly_buf[GPS_ASSEMBLY_BUF_SIZE_APP]; // Buffer to accumulate GPS data into complete NMEA sentences.
    uint16_t gps_assembly_len;                        // Current length of data in gps_assembly_buf.
} prog_state_t;

#endif // MAIN_H