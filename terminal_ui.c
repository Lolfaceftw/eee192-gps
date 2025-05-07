/**
 * @file      terminal_ui.c
 * @brief     Implementation of terminal user interface handling functions.
 *
 * This file contains the logic for displaying information on a connected
 * serial terminal. It uses ANSI escape codes for screen manipulation like
 * clearing the screen, positioning the cursor, and clearing lines.
 * It manages the transmission of a startup banner, raw GPS data (for debugging),
 * and formatted, parsed GPS data.
 *
 * @author    Alberto de Villa <alberto.de.villa@eee.upd.edu.ph> (Original Structure)
 * @author    Christian Klein C. Ramos (Docstrings, Comments, Adherence to Project Guidelines)
 * @date      07 May 2025
 */

#include "terminal_ui.h" // Corresponding header file for this module
#include "main.h"        // For prog_state_t definition, flags like PROG_FLAG_BANNER_PENDING, PROG_FLAG_TX_BUFFER_BUSY, PROG_FLAG_PARSED_GPGLL_PENDING, and TX_BUFFER_SIZE_APP
#include <stdio.h>       // For snprintf, used to format strings with ANSI codes
#include <string.h>      // For strlen (calculating string length) and memcpy (copying data to tx buffer)
#include "platform.h"    // For platform_usart_cdc_tx_busy (checking transmitter status) and platform_usart_cdc_tx_async (initiating transmission)

// --- ANSI Escape Codes for Terminal Control (Internal to UI module) ---
// These are defined as static const char arrays to ensure they exist in read-only memory
// and can be reliably pointed to or used in string operations.

/** @brief ANSI escape sequence to reset all terminal formatting to default. */
static const char ANSI_RESET_FORMAT[]           = "\033[0m";
/** @brief ANSI escape sequence to clear the entire terminal screen. */
static const char ANSI_CLEAR_SCREEN[]           = "\033[2J";
/** @brief ANSI escape sequence to move the cursor to the home position (top-left, typically 1;1). */
static const char ANSI_CURSOR_HOME[]            = "\033[1;1H";
/** @brief ANSI escape sequence to move the cursor to a specific position (Line 11, Column 7) for data display. */
static const char ANSI_CURSOR_TO_DATA_LINE[]    = "\033[11;7H";
/** @brief ANSI escape sequence to clear the line from the current cursor position to the end of the line. */
static const char ANSI_CLEAR_LINE_FROM_CURSOR[] = "\033[K";

/**
 * @brief Complete banner message string, including all necessary ANSI codes for screen setup
 * and initial cursor positioning. This string is intended for direct transmission
 * from its read-only memory location.
 */
static const char complete_banner_for_direct_send[] =
    "\033[0m"           // Reset terminal formatting
    "\033[2J"           // Clear the entire screen
    "\033[1;1H"         // Move cursor to top-left (Line 1, Column 1)
    // --- Banner Text Content ---
    "+--------------------------------------------------------------------+\r\n" // Line 1
    "| EEE 192: Electrical and Electronics Engineering Laboratory VI      |\r\n" // Line 2
    "|          Academic Year 2024-2025, Semester 2                       |\r\n" // Line 3
    "|                                                                    |\r\n" // Line 4
    "| Sensor: GPS Module                                                 |\r\n" // Line 5
    "|                                                                    |\r\n" // Line 6
    "| Author:  De Villa, Estrada, & Ramos (EEE 192 2S)                   |\r\n" // Line 7
    "| Date:    2025                                                      |\r\n" // Line 8
    "+--------------------------------------------------------------------+\r\n" // Line 9
    "\r\n"                                                                       // Line 10 (blank)
    "Data: "                                                                   // Line 11, prefix for data
    // --- End of Banner Text Content ---
    "\033[11;7H";   // Position cursor to Line 11, Column 7 (after "Data: ") for subsequent data output


/**
 * @brief Handles the transmission of the application banner to the terminal.
 *
 * This function checks if the banner display is pending and if the communication
 * channel (USART CDC) is free. If both conditions are met, it initiates an
 * asynchronous transmission of a predefined, complete banner string.
 * The banner string includes ANSI escape codes for resetting the terminal,
 * clearing the screen, and positioning the cursor.
 *
 * The function manages the `PROG_FLAG_BANNER_PENDING` and `PROG_FLAG_TX_BUFFER_BUSY`
 * flags in the program state (`ps`) to coordinate the transmission.
 *
 * @param[in,out] ps Pointer to the program state structure (`prog_state_t`).
 *                   The `flags` member is checked and modified to manage banner and TX states.
 *                   The `tx_desc` member is configured for the transmission.
 *                   The `banner_has_been_displayed_this_session` member is set to true
 *                   upon successful transmission initiation.
 */
void ui_handle_banner_transmission(struct prog_state_type *ps) {
    // Pre-conditions for sending the banner:
    // 1. The banner display must be pending.
    if (!(ps->flags & PROG_FLAG_BANNER_PENDING)) return; // If not pending, do nothing.
    // 2. The platform's USART CDC transmitter must not be busy with a previous operation.
    if (platform_usart_cdc_tx_busy()) return; // If hardware is busy, retry later.
    // 3. The application-level transmit buffer/descriptor must not be claimed by another operation.
    if (ps->flags & PROG_FLAG_TX_BUFFER_BUSY) return; // If ps->tx_desc is already in use, retry later.

    // Prepare to send the banner directly from its static const memory.
    // The program state's `tx_buf` is NOT used for the banner itself to save copying.
    // The `tx_desc.buf` points directly to the `complete_banner_for_direct_send` string.
    // A cast to (char*) might be needed if the platform API expects a non-const char pointer,
    // though sending const data is generally safe if the underlying driver handles it correctly.
    ps->tx_desc.buf = (char*)complete_banner_for_direct_send;
    // Calculate the length of the banner string (excluding the null terminator).
    ps->tx_desc.len = sizeof(complete_banner_for_direct_send) - 1; 
    
    // Claim the application-level transmission descriptor/flag before attempting the async send.
    // This indicates that `ps->tx_desc` is now being prepared or used for a transmission.
    ps->flags |= PROG_FLAG_TX_BUFFER_BUSY;

    // Attempt to start the asynchronous transmission of the banner.
    // The '1' indicates a single buffer descriptor is being used.
    if (platform_usart_cdc_tx_async(&ps->tx_desc, 1)) {
        // Asynchronous transmission was successfully started by the platform layer.
        // The hardware/HAL now manages the `tx_desc` until transmission completes.
        // Clear the banner pending flag as the request is now being handled.
        // Clear the application-level TX busy flag as this specific operation is now handed off.
        ps->flags &= ~(PROG_FLAG_BANNER_PENDING | PROG_FLAG_TX_BUFFER_BUSY);
        // Mark that the banner has been displayed in this session.
        ps->banner_has_been_displayed_this_session = true;
    } else {
        // Failed to start the asynchronous transmission (e.g., platform's internal queue might be full).
        // The `PROG_FLAG_TX_BUFFER_BUSY` remains set because `ps->tx_desc` is still "claimed"
        // by this pending banner operation from the application's perspective.
        // The `PROG_FLAG_BANNER_PENDING` also remains set.
        // The operation will be retried in a subsequent call to this function.
    }
}

/**
 * @brief Handles the transmission of formatted, parsed GPGLL data to the terminal.
 *
 * This function prepares a string for display, which includes the parsed GPGLL data
 * (e.g., "Time | Lon | Lat") along with ANSI escape codes for cursor positioning
 * and line clearing. The exact ANSI codes used depend on whether the application
 * banner has already been displayed and whether debug mode (raw GPS printing) is active.
 *
 * If the USART CDC transmitter is free, it formats the output into `ps->tx_buf`,
 * then initiates an asynchronous transmission. On success, it clears flags related
 * to pending parsed GPGLL data and TX buffer busy status, and also clears the
 * source buffer (`gpgll_to_parse_storage`) that held the raw NMEA sentence.
 *
 * @param[in,out] ps Pointer to the program state structure (`prog_state_t`).
 *                   The `flags` member is checked/modified. `tx_buf` is used to prepare the output,
 *                   and `tx_desc` is configured for transmission.
 *                   `banner_has_been_displayed_this_session` is checked.
 * @param[in]     parsed_data_str A C-string containing the human-readable parsed GPGLL data
 *                                (e.g., "23:59:59 | 079d59.0350'W | 40d43.9620'N").
 * @param[in,out] gpgll_to_parse_storage Pointer to the character buffer (typically in `main.c`)
 *                                       that held the raw GPGLL NMEA sentence which was parsed
 *                                       to produce `parsed_data_str`. This buffer is cleared
 *                                       (first character set to '\0') if this function successfully
 *                                       initiates the transmission of the parsed data.
 * @param[in]     debug_mode_raw_gps A boolean flag. If true, ANSI codes for specific line
 *                                   updating are omitted, and data is printed sequentially,
 *                                   allowing the terminal to scroll. If false, ANSI codes are used
 *                                   to update a fixed line on the terminal.
 */
void ui_handle_parsed_data_transmission(struct prog_state_type *ps,
                                        const char *parsed_data_str,
                                        char* gpgll_to_parse_storage, 
                                        bool debug_mode_raw_gps) {
    // Pre-conditions for sending parsed data:
    // 1. The platform's USART CDC transmitter must not be busy.
    if (platform_usart_cdc_tx_busy()) return; // If hardware is busy, retry later.
    // 2. The application-level transmit buffer/descriptor must not be claimed.
    if (ps->flags & PROG_FLAG_TX_BUFFER_BUSY) return; // If ps->tx_buf/tx_desc is in use, retry later.

    int final_output_len = 0; // To store the length of the string formatted by snprintf.

    // Construct the output string in the program state's transmit buffer (`ps->tx_buf`),
    // including ANSI escape codes for cursor positioning and line clearing as needed.
    if (!debug_mode_raw_gps) {
        // Normal mode (not raw debug mode): update a specific line on the terminal.
        if (ps->banner_has_been_displayed_this_session) {
            // If the banner has been displayed, move the cursor to the designated data line,
            // print the parsed data, and then clear the rest of that line.
            final_output_len = snprintf(ps->tx_buf, TX_BUFFER_SIZE_APP,
                                        "%s%s%s", // Format: <Move Cursor><Data String><Clear Rest of Line>
                                        ANSI_CURSOR_TO_DATA_LINE,    // Moves cursor to Line 11, Column 7
                                        parsed_data_str,             // The actual parsed data
                                        ANSI_CLEAR_LINE_FROM_CURSOR);// Clears from cursor to end of line
        } else {
            // If the banner has not yet been shown (or its transmission failed),
            // update from the home position (top-left) as a fallback.
            final_output_len = snprintf(ps->tx_buf, TX_BUFFER_SIZE_APP,
                                        "%s%s%s", // Format: <Move Cursor><Data String><Clear Rest of Line>
                                        ANSI_CURSOR_HOME,            // Moves cursor to Line 1, Column 1
                                        parsed_data_str,             // The actual parsed data
                                        ANSI_CLEAR_LINE_FROM_CURSOR);// Clears from cursor to end of line
        }
    } else {
        // Debug mode (raw GPS printing is active): simply append the parsed data.
        // The terminal will scroll naturally. No special ANSI cursor positioning is used here.
        final_output_len = snprintf(ps->tx_buf, TX_BUFFER_SIZE_APP, "%s", parsed_data_str);
    }

    // Check if snprintf was successful and the formatted output fits within the transmit buffer.
    // `final_output_len > 0` means snprintf didn't encounter an error.
    // `(size_t)final_output_len < TX_BUFFER_SIZE_APP` means the string was not truncated.
    if (final_output_len > 0 && (size_t)final_output_len < TX_BUFFER_SIZE_APP) {
        // The output string is valid and fits. Prepare for transmission.
        ps->tx_desc.buf = ps->tx_buf; // Point the transmission descriptor to our prepared buffer.
        ps->tx_desc.len = final_output_len; // Set the length of data to transmit.
        
        ps->flags |= PROG_FLAG_TX_BUFFER_BUSY; // Claim the application-level TX descriptor/flag.

        // Attempt to start the asynchronous transmission.
        if (platform_usart_cdc_tx_async(&ps->tx_desc, 1)) {
            // Asynchronous transmission successfully started.
            // Clear the flag indicating parsed GPGLL data is pending (as it's now being sent).
            // Clear the application-level TX busy flag.
            ps->flags &= ~(PROG_FLAG_PARSED_GPGLL_PENDING | PROG_FLAG_TX_BUFFER_BUSY);
            
            // If a source buffer for the raw GPGLL sentence was provided...
            if (gpgll_to_parse_storage) { 
                gpgll_to_parse_storage[0] = '\0'; // ...clear it, as its content has been processed and sent.
            }
        } else {
            // Failed to start asynchronous transmission (e.g., platform's internal queue full).
            // `PROG_FLAG_TX_BUFFER_BUSY` remains set because `ps->tx_desc` is still "claimed".
            // `PROG_FLAG_PARSED_GPGLL_PENDING` also remains set.
            // The operation will be retried in a subsequent call.
        }
    } else {
        // An error occurred with snprintf (e.g., `final_output_len <= 0`),
        // or the formatted output was too long and truncated (`final_output_len >= TX_BUFFER_SIZE_APP`).
        // In this case, we cannot reliably send this data.
        // Clear the pending flag for parsed GPGLL data to prevent retrying a bad format operation.
        ps->flags &= ~PROG_FLAG_PARSED_GPGLL_PENDING;
        // If a source buffer was provided, clear it as well.
        if (gpgll_to_parse_storage) {
            gpgll_to_parse_storage[0] = '\0';
        }
        // Optional: Consider logging this failure if a debugging mechanism is available.
        // e.g., log_error("Parsed data formatting error or buffer overflow.");
    }
}

/**
 * @brief Handles the transmission of raw data (e.g., NMEA sentences) to the terminal.
 *
 * This function is typically used in debug modes to display unprocessed data streams.
 * If the USART CDC transmitter is free and the provided raw data fits into the
 * application's transmit buffer (`ps->tx_buf`), it copies the data and initiates
 * an asynchronous transmission.
 *
 * @param[in,out] ps Pointer to the program state structure (`prog_state_t`).
 *                   The `flags` member is checked/modified. `tx_buf` is used to stage the data,
 *                   and `tx_desc` is configured for transmission.
 * @param[in]     raw_data_str Pointer to the raw data string to be transmitted. This data
 *                             does not need to be null-terminated if `raw_data_len` is accurate.
 * @param[in]     raw_data_len The length of the raw data string in bytes.
 * @return        `true` if the asynchronous transmission was successfully initiated.
 * @return        `false` otherwise (e.g., transmitter busy, data too long for `ps->tx_buf`,
 *                or platform failed to start async send).
 */
bool ui_handle_raw_data_transmission(struct prog_state_type *ps,
                                     const char *raw_data_str,
                                     size_t raw_data_len) {
    // Pre-conditions for sending raw data:
    // 1. The platform's USART CDC transmitter must not be busy.
    if (platform_usart_cdc_tx_busy()) return false; // If hardware is busy, indicate failure to initiate.
    // 2. The application-level transmit buffer/descriptor must not be claimed.
    if (ps->flags & PROG_FLAG_TX_BUFFER_BUSY) return false; // If ps->tx_buf/tx_desc is in use, indicate failure.

    // Ensure the raw data to be sent can fit into the program state's transmit buffer (`ps->tx_buf`).
    if (raw_data_len < TX_BUFFER_SIZE_APP) { // Check against the defined buffer size.
        // Copy the raw data into the transmit buffer.
        memcpy(ps->tx_buf, raw_data_str, raw_data_len);
        
        // Null termination of `ps->tx_buf` after `memcpy` is good practice if any subsequent code
        // might inadvertently treat `ps->tx_buf` as a C-string up to its full capacity.
        // However, `platform_usart_cdc_tx_async` uses an explicit length (`raw_data_len`),
        // so null termination is not strictly required for the transmission itself.
        // The condition `raw_data_len < TX_BUFFER_SIZE_APP` ensures there's at least one byte
        // of space for a null terminator if `raw_data_len` was `TX_BUFFER_SIZE_APP - 1`.
        // For safety, one might add: ps->tx_buf[raw_data_len] = '\0'; (if space is guaranteed).
        // This line was commented out in the original, so it remains commented.
        // ps->tx_buf[raw_data_len] = '\0'; 

        // Configure the transmission descriptor.
        ps->tx_desc.buf = ps->tx_buf;       // Point to the data in our transmit buffer.
        ps->tx_desc.len = raw_data_len;     // Set the length of data to send.
        
        ps->flags |= PROG_FLAG_TX_BUFFER_BUSY; // Claim the application-level TX descriptor/flag.

        // Attempt to start the asynchronous transmission.
        if (platform_usart_cdc_tx_async(&ps->tx_desc, 1)) {
            // Asynchronous transmission successfully started.
            // Clear the application-level TX busy flag as this specific operation is now handed off.
            ps->flags &= ~PROG_FLAG_TX_BUFFER_BUSY;
            return true; // Indicate successful initiation of transmission.
        } else {
            // Failed to start asynchronous transmission (e.g., platform's internal queue full).
            // `PROG_FLAG_TX_BUFFER_BUSY` remains set because `ps->tx_desc` is still "claimed".
            return false; // Indicate failure to initiate transmission.
        }
    } else {
        // The provided raw data is too long to fit into `ps->tx_buf`.
        // Optional: Consider logging this failure if a debugging mechanism is available.
        // e.g., log_error("Raw data too long for transmit buffer.");
        return false; // Indicate failure due to buffer size constraint.
    }
}
