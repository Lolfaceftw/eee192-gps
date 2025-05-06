#include "terminal_ui.h" 
#include "main.h"        // For prog_state_t definition, flags, TX_BUFFER_SIZE_APP
#include <stdio.h>       // For snprintf
#include <string.h>      // For strlen, memcpy
#include "platform.h"     // For platform_usart_cdc_tx_busy, platform_usart_cdc_tx_async

// --- ANSI Escape Codes for Terminal Control (Internal to UI module) ---
// These are defined as static const char arrays to ensure they exist in memory
// and can be reliably pointed to or used in string operations.
static const char ANSI_RESET_FORMAT[]           = "\033[0m";
static const char ANSI_CLEAR_SCREEN[]           = "\033[2J";
static const char ANSI_CURSOR_HOME[]            = "\033[1;1H";
static const char ANSI_CURSOR_TO_DATA_LINE[]    = "\033[11;7H"; // Positions cursor to Line 11, Column 7
static const char ANSI_CLEAR_LINE_FROM_CURSOR[] = "\033[K";     // Clears from cursor to end of line

/**
 * @brief Complete banner message string, including all necessary ANSI codes for screen setup
 * and initial cursor positioning. This string is intended for direct transmission.
 */
static const char complete_banner_for_direct_send[] =
    "\033[0m"           // Reset terminal formatting
    "\033[2J"           // Clear the entire screen
    "\033[1;1H"         // Move cursor to top-left
    // --- Banner Text Content ---
    "+--------------------------------------------------------------------+\r\n"
    "| EEE 192: Electrical and Electronics Engineering Laboratory VI      |\r\n"
    "|          Academic Year 2024-2025, Semester 2                       |\r\n"
    "|                                                                    |\r\n"
    "| Sensor: GPS Module                                                 |\r\n"
    "|                                                                    |\r\n"
    "| Author:  De Villa, Estrada, & Ramos (EEE 192 2S)                   |\r\n"
    "| Date:    2025                                                      |\r\n"
    "+--------------------------------------------------------------------+\r\n"
    "\r\n"
    "Data: "
    // --- End of Banner Text Content ---
    "\033[11;7H";   // Position cursor for subsequent data output


void ui_handle_banner_transmission(struct prog_state_type *ps) {
    // Pre-conditions: banner must be pending, and transmitter must be free for a new operation.
    if (!(ps->flags & PROG_FLAG_BANNER_PENDING)) return;
    if (platform_usart_cdc_tx_busy()) return;
    if (ps->flags & PROG_FLAG_TX_BUFFER_BUSY) return; // ps->tx_desc is already claimed

    // Prepare to send the banner directly from its static const memory.
    // ps->tx_buf is NOT used for the banner itself.
    ps->tx_desc.buf = (char*)complete_banner_for_direct_send; // Cast to char* if platform API expects non-const
    ps->tx_desc.len = sizeof(complete_banner_for_direct_send) - 1; // Get length of the char array
    
    // Claim the transmission descriptor/flag before attempting the async send.
    ps->flags |= PROG_FLAG_TX_BUFFER_BUSY;

    if (platform_usart_cdc_tx_async(&ps->tx_desc, 1)) {
        // Asynchronous transmission successfully started.
        // Release the PROG_FLAG_TX_BUFFER_BUSY as the hardware/HAL now manages tx_desc.
        // Clear the banner pending flag.
        ps->flags &= ~(PROG_FLAG_BANNER_PENDING | PROG_FLAG_TX_BUFFER_BUSY);
        ps->banner_has_been_displayed_this_session = true;
    } else {
        // Failed to start async transmission (e.g., HAL queue full).
        // PROG_FLAG_TX_BUFFER_BUSY remains set because we still hold the intent to use tx_desc.
        // PROG_FLAG_BANNER_PENDING also remains set. The operation will be retried.
    }
}

void ui_handle_parsed_data_transmission(struct prog_state_type *ps,
                                        const char *parsed_data_str,
                                        char* gpgll_to_parse_storage, // Buffer in main.c holding the raw NMEA GPGLL
                                        bool debug_mode_raw_gps) {
    // Pre-conditions: transmitter must be free for a new operation.
    if (platform_usart_cdc_tx_busy()) return;
    if (ps->flags & PROG_FLAG_TX_BUFFER_BUSY) return; // ps->tx_desc and ps->tx_buf are already claimed

    int final_output_len = 0;

    // Construct the output string in ps->tx_buf, including ANSI codes for cursor positioning.
    if (!debug_mode_raw_gps) {
        // Normal mode: update a specific line on the terminal.
        if (ps->banner_has_been_displayed_this_session) {
            // Banner has been shown, update the designated data line.
            final_output_len = snprintf(ps->tx_buf, TX_BUFFER_SIZE_APP,
                                        "%s%s%s", // Format: <Move Cursor><Data><Clear Rest of Line>
                                        ANSI_CURSOR_TO_DATA_LINE,
                                        parsed_data_str,
                                        ANSI_CLEAR_LINE_FROM_CURSOR);
        } else {
            // Banner not yet shown (or failed); update from the home position.
            final_output_len = snprintf(ps->tx_buf, TX_BUFFER_SIZE_APP,
                                        "%s%s%s", // Format: <Move Cursor><Data><Clear Rest of Line>
                                        ANSI_CURSOR_HOME,
                                        parsed_data_str,
                                        ANSI_CLEAR_LINE_FROM_CURSOR);
        }
    } else {
        // Debug mode: simply append the parsed data, allowing terminal to scroll.
        final_output_len = snprintf(ps->tx_buf, TX_BUFFER_SIZE_APP, "%s", parsed_data_str);
    }

    // Check if snprintf was successful and the output fits the buffer.
    if (final_output_len > 0 && (size_t)final_output_len < TX_BUFFER_SIZE_APP) {
        ps->tx_desc.buf = ps->tx_buf; // Point descriptor to our prepared buffer.
        ps->tx_desc.len = final_output_len;
        ps->flags |= PROG_FLAG_TX_BUFFER_BUSY; // Claim the transmission descriptor/flag.

        if (platform_usart_cdc_tx_async(&ps->tx_desc, 1)) {
            // Async transmission started. Release flags.
            ps->flags &= ~(PROG_FLAG_PARSED_GPGLL_PENDING | PROG_FLAG_TX_BUFFER_BUSY);
            if (gpgll_to_parse_storage) { // If a source buffer was provided...
                gpgll_to_parse_storage[0] = '\0'; // ...clear it, as its content is now processed.
            }
        } else {
            // Failed to start async transmission. PROG_FLAG_TX_BUFFER_BUSY remains set.
            // PROG_FLAG_PARSED_GPGLL_PENDING also remains set. Will retry.
        }
    } else {
        // snprintf error (final_output_len <= 0) or output was truncated (final_output_len >= TX_BUFFER_SIZE_APP).
        // Cannot reliably send this data. Clear the pending flag to prevent retrying a bad format.
        ps->flags &= ~PROG_FLAG_PARSED_GPGLL_PENDING;
        if (gpgll_to_parse_storage) {
            gpgll_to_parse_storage[0] = '\0';
        }
        // Consider logging this failure: e.g., log_error("Parsed data formatting/buffer error.");
    }
}

bool ui_handle_raw_data_transmission(struct prog_state_type *ps,
                                     const char *raw_data_str,
                                     size_t raw_data_len) {
    // Pre-conditions: transmitter must be free for a new operation.
    if (platform_usart_cdc_tx_busy()) return false;
    if (ps->flags & PROG_FLAG_TX_BUFFER_BUSY) return false; // ps->tx_desc and ps->tx_buf are already claimed

    // Ensure the raw data fits into ps->tx_buf.
    if (raw_data_len < TX_BUFFER_SIZE_APP) {
        memcpy(ps->tx_buf, raw_data_str, raw_data_len);
        // Null termination of ps->tx_buf after memcpy is good practice if subsequent code
        // might treat it as a C-string, though platform_usart_cdc_tx_async uses explicit length.
        // If raw_data_len is exactly TX_BUFFER_SIZE_APP-1, this would overwrite the last char.
        // However, the check is raw_data_len < TX_BUFFER_SIZE_APP, so there's space for null if needed.
        // For sending by length, it's not strictly required.
        // ps->tx_buf[raw_data_len] = '\0'; 

        ps->tx_desc.buf = ps->tx_buf;
        ps->tx_desc.len = raw_data_len;
        ps->flags |= PROG_FLAG_TX_BUFFER_BUSY; // Claim the transmission descriptor/flag.

        if (platform_usart_cdc_tx_async(&ps->tx_desc, 1)) {
            // Async transmission started. Release general TX busy flag.
            ps->flags &= ~PROG_FLAG_TX_BUFFER_BUSY;
            return true; // Indicate successful initiation.
        } else {
            // Failed to start async transmission. PROG_FLAG_TX_BUFFER_BUSY remains set.
            return false; // Indicate failure to initiate.
        }
    } else {
        // Raw data is too long for the transmit buffer.
        // Consider logging this failure: e.g., log_error("Raw data too long for tx_buf.");
        return false;
    }
}