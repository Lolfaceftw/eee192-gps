/**
 * @file      main.c
 * @brief     Main application file for the GPS Data Logger and Parser.
 *
 * Orchestrates GPS data reception, NMEA sentence processing (via nmea_parser module),
 * and terminal output (via terminal_ui module). Manages overall application state.
 * The definition of the program state (prog_state_t) and its associated flags
 * are now located in main.h.
 *
 * @author    Alberto de Villa <alberto.de.villa@eee.upd.edu.ph> (Original)
 * @author    Estrada (Supplemented by EEE 158 AY 24-25 1S)
 * @author    Christian Klein C. Ramos (Supplemented by EEE 192 AY 24-25 2S, Refactored, Modularized, Docstrings & Comments Added)
 * @date      07 May 2025 // Last significant modification date reflecting these changes
 */

// Standard C and Microcontroller specific includes
#include <xc.h>        // For microcontroller-specific definitions (e.g., PORT_SEC_REGS)
#include <string.h>    // For string manipulation functions (memset, memcpy, strstr, strncmp, strncpy, strlen)
#include <stdio.h>     // For snprintf (used by nmea_parser and terminal_ui, though not directly in main.c)
#include <stdbool.h>   // For boolean type (true, false)
#include <stdlib.h>    // For general utilities (not explicitly used here but often included)

// Project-specific includes
#include "main.h"          // For prog_state_t, program flags, and core app constants
#include "platform.h"      // For platform initialization and hardware interaction functions
#include "nmea_parser.h"   // For NMEA sentence parsing logic
#include "terminal_ui.h"   // For UI handling functions (banner, raw data, parsed data display)

// --- Application Configuration ---
// This constant is used by main logic and passed to UI functions.
// Determines if raw GPS NMEA sentences are printed to the terminal.
static const bool DEBUG_MODE_PRINT_RAW_GPS = true;

// --- Application Constants (main.c specific, if any beyond main.h) ---
// NMEA identification prefixes are specific to main.c's pre-filtering logic.
#define APP_NMEA_LINE_ENDING "\r\n" // Standard NMEA sentence line ending
#define APP_NMEA_GPGLL_PREFIX "$GPGLL," // Prefix for GPGLL NMEA sentences
#define APP_NMEA_GPGLL_PREFIX_LEN (sizeof(APP_NMEA_GPGLL_PREFIX) - 1) // Length of GPGLL prefix, excluding null terminator

// LED Indicator (specific to main.c's direct hardware interaction for activity indication)
#define LED_ACTIVITY_PORT_GROUP (PORT_SEC_REGS->GROUP[0]) // Port group for the activity LED
#define LED_ACTIVITY_PIN        (1 << 15)                 // Pin mask for the activity LED (PA15)

// prog_state_t typedef and PROG_FLAG_* definitions are in main.h

// --- FAKE GPS DATA FOR DEBUGGING ---
// Controls whether the application uses predefined fake GPGLL sentences or real GPS data.
#define USE_FAKE_GPS_DATA false // Set to true to use fake data, false for real GPS
                               // IMPORTANT: If true, real GPS data processing for GPGLL will be overridden

#if USE_FAKE_GPS_DATA
// Array of fake GPGLL sentences to cycle through for debugging purposes.
static const char* fake_gpgll_sentences[] = {
    "$GPGLL,4043.9620,N,07959.0350,W,235959.00,A,A*77", // Example: Pittsburgh, PA, USA (Night)
    "$GPGLL,3403.7658,S,15052.9787,E,123045.10,A,A*6C", // Example: Sydney, Australia (Day)
    "$GPGLL,4807.038,N,01131.000,E,104820.22,A,A*4D",   // Example: Munich, Germany
    "$GPGLL,2237.0000,N,11408.0000,E,081530.00,A,A*7A", // Example: Hong Kong
    "$GPGLL,,,,,123519.00,V,N*4D",                     // Example: Invalid/No Fix, time only
    "$GPGLL,5130.0000,N,00007.0000,W,140000.00,A,A*78"  // Example: Greenwich, London
};
// Number of sentences in the fake_gpgll_sentences array.
static const int num_fake_sentences = sizeof(fake_gpgll_sentences) / sizeof(fake_gpgll_sentences[0]);
#endif
// --- End FAKE GPS DATA ---

// --- Static Function Prototypes (main.c internal logic) ---

/**
 * @brief Handles platform-specific events, such as button presses.
 *
 * Calls `platform_do_loop_one()` for general platform processing and checks for
 * pushbutton events. If an on-board button press is detected, it sets a flag
 * to indicate that the application banner should be displayed.
 *
 * @param[in,out] ps Pointer to the program state structure.
 */
static void handle_platform_events(prog_state_t *ps);

/**
 * @brief Handles the reception of data from the GPS module.
 *
 * Checks if new data has been received via the GPS USART interface. If so,
 * it appends the received data to an assembly buffer (`ps->gps_assembly_buf`).
 * If a complete NMEA line ending (`APP_NMEA_LINE_ENDING`) is found in the
 * assembly buffer, it sets a flag (`PROG_FLAG_GPS_UPDATE_PENDING`) to indicate
 * that GPS data is ready for processing. It also re-initiates asynchronous
 * reception for more data. An LED is blinked to indicate activity.
 *
 * @param[in,out] ps Pointer to the program state structure.
 */
static void handle_gps_reception(prog_state_t *ps);

/**
 * @brief Processes complete NMEA sentences from the GPS assembly buffer.
 *
 * If the `PROG_FLAG_GPS_UPDATE_PENDING` flag is set, this function searches for
 * the first complete NMEA sentence in `ps->gps_assembly_buf`.
 * If `DEBUG_MODE_PRINT_RAW_GPS` is true, it attempts to transmit the raw sentence
 * via the UI module.
 * If the sentence is a GPGLL sentence and no other GPGLL is pending parsing,
 * it copies the GPGLL sentence content to `gpgll_storage_buf` and sets the
 * `PROG_FLAG_PARSED_GPGLL_PENDING` flag.
 * Finally, it removes the processed sentence from the assembly buffer.
 *
 * @param[in,out] ps Pointer to the program state structure.
 * @param[out]    gpgll_storage_buf Buffer to store the content of a GPGLL sentence if found.
 * @param[in]     gpgll_storage_size Size of the `gpgll_storage_buf`.
 */
static void handle_gps_sentence_processing(prog_state_t *ps, char* gpgll_storage_buf, size_t gpgll_storage_size);

/**
 * @brief Handles parsing of a stored GPGLL sentence and requests its display.
 *
 * If the `PROG_FLAG_PARSED_GPGLL_PENDING` flag is set and a GPGLL sentence
 * is available in `gpgll_to_parse_storage`, this function calls
 * `nmea_parse_gpgll_and_format()` to parse and format the sentence.
 * If parsing is successful, it requests the UI module to transmit the formatted data.
 * The UI module is responsible for clearing the `PROG_FLAG_PARSED_GPGLL_PENDING`
 * flag upon successful transmission initiation.
 *
 * @param[in,out] ps Pointer to the program state structure.
 * @param[in,out] gpgll_to_parse_storage Buffer containing the GPGLL sentence to parse.
 *                                       This buffer will be cleared by the UI module upon successful handling.
 */
static void handle_gpgll_parsing_and_request_display(prog_state_t *ps, char* gpgll_to_parse_storage);

/**
 * @brief Removes a processed NMEA line from the beginning of the GPS assembly buffer.
 *
 * Shifts the content of `ps->gps_assembly_buf` to remove the first `line_len_with_crlf`
 * bytes. Updates `ps->gps_assembly_len` and ensures the buffer remains null-terminated.
 * If no more line endings are found in the buffer after removal, it clears the
 * `PROG_FLAG_GPS_UPDATE_PENDING` flag.
 *
 * @param[in,out] ps Pointer to the program state structure.
 * @param[in]     line_len_with_crlf The total length of the line to remove, including its CRLF ending.
 */
static void remove_line_from_gps_assembly_buffer(prog_state_t *ps, int line_len_with_crlf);

/**
 * @brief Initializes the program state and hardware platform.
 *
 * This function is called once at the start of the application. It performs:
 * 1. Zero-initialization of the program state structure (`ps`).
 * 2. Sets initial state flags (e.g., `banner_has_been_displayed_this_session`).
 * 3. Calls `platform_init()` to initialize microcontroller peripherals and clocks.
 * 4. Sets up asynchronous data reception for both the CDC (terminal) and GPS USART interfaces.
 *
 * @param[out] ps Pointer to the program state structure to be initialized.
 */
static void prog_setup(prog_state_t *ps)
{
    // Initialize the entire program state structure to zeros.
	memset(ps, 0, sizeof(prog_state_t));
    // Set initial flag indicating the banner has not yet been shown in this run.
    ps->banner_has_been_displayed_this_session = false;

	// Initialize the hardware platform (clocks, GPIOs, SysTick, etc.).
	platform_init();
	
    // Configure the asynchronous reception descriptor for the CDC (USB Virtual COM Port).
	ps->cdc_rx_desc.buf     = ps->cdc_rx_buf;          // Buffer to store received CDC data.
	ps->cdc_rx_desc.max_len = RX_BUFFER_CDC_SIZE_APP;  // Maximum length of the CDC receive buffer.
	// Start asynchronous reception on the CDC USART.
	platform_usart_cdc_rx_async(&ps->cdc_rx_desc);
    
    // Configure the asynchronous reception descriptor for the GPS USART.
    ps->gps_rx_desc.buf     = ps->gps_rx_buf;          // Buffer to store received GPS data.
    ps->gps_rx_desc.max_len = RX_BUFFER_GPS_SIZE_APP;  // Maximum length of the GPS receive buffer.
    // Start asynchronous reception on the GPS USART.
    gps_platform_usart_cdc_rx_async(&ps->gps_rx_desc);
}

/**
 * @brief Handles platform-specific events, such as button presses.
 *
 * This function calls `platform_do_loop_one()` to allow the platform layer
 * to perform any necessary periodic tasks (e.g., servicing timers or low-level drivers).
 * It then checks for events from the on-board pushbutton using `platform_pb_get_event()`.
 * If a press event is detected, it sets the `PROG_FLAG_BANNER_PENDING` flag in the
 * program state, signaling that the application banner should be displayed.
 *
 * @param[in,out] ps Pointer to the program state structure. The `flags` member may be modified.
 */
static void handle_platform_events(prog_state_t *ps) {
    // Perform one iteration of platform-level event processing.
    platform_do_loop_one();
    // Get any pushbutton events that have occurred since the last call.
    uint16_t pb_event = platform_pb_get_event();

    // Check if the on-board button press event occurred.
	if ((pb_event & PLATFORM_PB_ONBOARD_PRESS) != 0) {
        // If pressed, set the flag to indicate the banner display is pending.
		ps->flags |= PROG_FLAG_BANNER_PENDING; // PROG_FLAG_* defined in main.h
	}
}

/**
 * @brief Handles the reception of data from the GPS module.
 *
 * This function checks the completion status of the asynchronous GPS data reception.
 * If data has been successfully received (`PLATFORM_USART_RX_COMPL_DATA`):
 * 1. Turns on an activity LED (PA15).
 * 2. Appends the newly received data from `ps->gps_rx_buf` to `ps->gps_assembly_buf`.
 * 3. Null-terminates the assembly buffer.
 * 4. If a complete NMEA line ending (`APP_NMEA_LINE_ENDING`) is detected in the
 *    assembly buffer and a GPS update is not already pending, it sets the
 *    `PROG_FLAG_GPS_UPDATE_PENDING` flag.
 * 5. If the assembly buffer becomes full, it's cleared to prevent overflow.
 * 6. Resets the reception completion type and re-initiates asynchronous GPS data reception.
 *
 * @param[in,out] ps Pointer to the program state structure. Modifies `gps_rx_desc`,
 *                   `gps_assembly_buf`, `gps_assembly_len`, and `flags`.
 */
static void handle_gps_reception(prog_state_t *ps) {
    // Check if the GPS USART reception has completed with data.
    if (ps->gps_rx_desc.compl_type == PLATFORM_USART_RX_COMPL_DATA) {
        // Turn on the activity LED to indicate data reception.
        LED_ACTIVITY_PORT_GROUP.PORT_OUTSET = LED_ACTIVITY_PIN;
        // Get the length of the received data.
        uint16_t received_len = ps->gps_rx_desc.compl_info.data_len;

        // Check if there's enough space in the assembly buffer for the new data.
        if (ps->gps_assembly_len + received_len < GPS_ASSEMBLY_BUF_SIZE_APP) { // Constant from main.h
            // Append the received data to the assembly buffer.
            memcpy(ps->gps_assembly_buf + ps->gps_assembly_len, ps->gps_rx_buf, received_len);
            // Update the length of data in the assembly buffer.
            ps->gps_assembly_len += received_len;
            // Null-terminate the assembly buffer.
            ps->gps_assembly_buf[ps->gps_assembly_len] = '\0';

            // Check if a complete NMEA line ending is present in the assembled data.
            if (strstr(ps->gps_assembly_buf, APP_NMEA_LINE_ENDING) != NULL) {
                // If a line ending is found and a GPS update is not already pending,
                // set the flag to indicate data is ready for processing.
                if (!(ps->flags & PROG_FLAG_GPS_UPDATE_PENDING)) {
                     ps->flags |= PROG_FLAG_GPS_UPDATE_PENDING;
                }
            }
        } else {
            // Assembly buffer would overflow; clear it to recover.
            ps->gps_assembly_len = 0;
            ps->gps_assembly_buf[0] = '\0';
        }
        // Reset the reception completion type.
        ps->gps_rx_desc.compl_type = PLATFORM_USART_RX_COMPL_NONE;
        // Re-initiate asynchronous reception for more GPS data.
        gps_platform_usart_cdc_rx_async(&ps->gps_rx_desc);
    }
}

/**
 * @brief Removes a processed NMEA line from the beginning of the GPS assembly buffer.
 *
 * This utility function is called after an NMEA sentence has been extracted and processed.
 * It shifts the remaining content of `ps->gps_assembly_buf` to the beginning, effectively
 * removing the first `line_len_with_crlf` bytes. It updates `ps->gps_assembly_len`
 * and ensures the buffer remains null-terminated. If, after removal, no more
 * NMEA line endings are found in the buffer, it clears the `PROG_FLAG_GPS_UPDATE_PENDING`
 * flag, as there are no more complete sentences ready for immediate processing.
 *
 * @param[in,out] ps Pointer to the program state structure. Modifies `gps_assembly_buf`,
 *                   `gps_assembly_len`, and potentially `flags`.
 * @param[in]     line_len_with_crlf The total length of the line to remove, including
 *                                   its CRLF (Carriage Return, Line Feed) ending.
 */
static void remove_line_from_gps_assembly_buffer(prog_state_t *ps, int line_len_with_crlf) {
    // Basic validation: if length to remove is invalid or exceeds current buffer content,
    // clear the buffer and the pending flag to prevent errors.
    if (line_len_with_crlf <= 0 || (uint16_t)line_len_with_crlf > ps->gps_assembly_len) {
        ps->gps_assembly_len = 0;
        ps->gps_assembly_buf[0] = '\0';
        ps->flags &= ~PROG_FLAG_GPS_UPDATE_PENDING; // Clear pending flag as buffer is now empty/invalid.
        return;
    }
    // Decrease the assembly buffer length by the length of the removed line.
    ps->gps_assembly_len -= line_len_with_crlf;
    // Move the remaining data in the buffer to the beginning.
    memmove(ps->gps_assembly_buf, ps->gps_assembly_buf + line_len_with_crlf, ps->gps_assembly_len);
    // Null-terminate the modified assembly buffer.
    ps->gps_assembly_buf[ps->gps_assembly_len] = '\0';

    // After removing a line, check if another complete line ending is still present.
    // If not, clear the GPS update pending flag.
    if (strstr(ps->gps_assembly_buf, APP_NMEA_LINE_ENDING) == NULL) {
        ps->flags &= ~PROG_FLAG_GPS_UPDATE_PENDING;
    }
}

/**
 * @brief Processes complete NMEA sentences from the GPS assembly buffer.
 *
 * This function is called when the `PROG_FLAG_GPS_UPDATE_PENDING` flag indicates
 * that one or more complete NMEA sentences might be in `ps->gps_assembly_buf`.
 * It extracts the first sentence, determines if it's a GPGLL sentence, and handles
 * its raw transmission (if `DEBUG_MODE_PRINT_RAW_GPS` is true) and/or storage for parsing.
 *
 * Operations:
 * 1. Returns if no GPS update is pending.
 * 2. Finds the first NMEA line ending (`APP_NMEA_LINE_ENDING`). If none, clears the pending flag.
 * 3. Extracts the content of the current sentence.
 * 4. Checks if the sentence is a GPGLL sentence.
 * 5. If `DEBUG_MODE_PRINT_RAW_GPS` is true:
 *    a. Attempts to transmit the raw sentence using `ui_handle_raw_data_transmission()`.
 *    b. If transmission is initiated and the sentence is GPGLL (and no other GPGLL is pending parsing),
 *       it stores the GPGLL sentence content in `gpgll_storage_buf` and sets `PROG_FLAG_PARSED_GPGLL_PENDING`.
 *    c. Removes the processed line from the assembly buffer.
 * 6. If not in debug mode or raw transmission was skipped/failed:
 *    a. If the sentence is GPGLL (and no other GPGLL is pending parsing), it stores it and sets the flag.
 *    b. Removes the processed line from the assembly buffer.
 *
 * @param[in,out] ps Pointer to the program state structure. Modifies `flags`, and indirectly
 *                   `gps_assembly_buf` and `gps_assembly_len` via `remove_line_from_gps_assembly_buffer`.
 * @param[out]    gpgll_storage_buf Buffer where the content of an identified GPGLL sentence is stored
 *                                  for later parsing. Its content is modified if a GPGLL sentence is found
 *                                  and conditions are met.
 * @param[in]     gpgll_storage_size The size of `gpgll_storage_buf`.
 */
static void handle_gps_sentence_processing(prog_state_t *ps, char* gpgll_storage_buf, size_t gpgll_storage_size) {
    // If no GPS update is pending, there's nothing to process.
    if (!(ps->flags & PROG_FLAG_GPS_UPDATE_PENDING)) return;

    // The UI functions called below (ui_handle_raw_data_transmission) will internally check
    // for transmission buffer busy status (PROG_FLAG_TX_BUFFER_BUSY and platform_usart_cdc_tx_busy()).
    // No need for redundant checks here before calling them.

    // Find the end of the first NMEA sentence in the assembly buffer.
    char* newline_ptr = strstr(ps->gps_assembly_buf, APP_NMEA_LINE_ENDING);
    if (newline_ptr == NULL) {
        // No complete line ending found, so no full sentence to process. Clear the flag.
        ps->flags &= ~PROG_FLAG_GPS_UPDATE_PENDING;
        return;
    }

    // Calculate the length of the NMEA sentence content (excluding CRLF).
    int sentence_content_len = newline_ptr - ps->gps_assembly_buf;
    // Calculate the total length of the line including the CRLF ending.
    int total_line_len = sentence_content_len + strlen(APP_NMEA_LINE_ENDING);

    // Temporary buffer to hold the content of the current NMEA sentence.
    char current_sentence_content[GPS_ASSEMBLY_BUF_SIZE_APP]; // Sized to max assembly buffer capacity.
    // Ensure the extracted sentence content length is not too large for the temporary buffer.
    if (sentence_content_len >= (int)sizeof(current_sentence_content)) {
        // Sentence content is too long, likely an error or buffer corruption.
        // Remove this problematic line from the assembly buffer and return.
        remove_line_from_gps_assembly_buffer(ps, total_line_len);
        return;
    }
    // Copy the sentence content (without CRLF) into the temporary buffer.
    memcpy(current_sentence_content, ps->gps_assembly_buf, sentence_content_len);
    // Null-terminate the copied sentence content.
    current_sentence_content[sentence_content_len] = '\0';

    // Check if the current sentence is a GPGLL sentence by comparing its prefix.
    bool is_gpgll = (strncmp(current_sentence_content, APP_NMEA_GPGLL_PREFIX, APP_NMEA_GPGLL_PREFIX_LEN) == 0);

    // If debug mode for printing raw GPS data is enabled:
    if (DEBUG_MODE_PRINT_RAW_GPS) {
        // Prepare the full raw line (including CRLF) from the assembly buffer to pass to the UI function.
        // Ensure the temporary buffer for sending is large enough.
        char raw_line_to_send[GPS_ASSEMBLY_BUF_SIZE_APP]; // Max possible line size from assembly.
        if(total_line_len < (int)sizeof(raw_line_to_send)){ // Check if it fits.
            // Copy the full line (content + CRLF) to the send buffer.
            memcpy(raw_line_to_send, ps->gps_assembly_buf, total_line_len);
            // Null termination of raw_line_to_send is not strictly needed here as
            // ui_handle_raw_data_transmission takes the length explicitly.

            // Attempt to transmit the raw line via the UI module.
            if (ui_handle_raw_data_transmission(ps, raw_line_to_send, total_line_len)) {
                // Raw send was successfully initiated by the UI module.
                // If the sent raw line was a GPGLL sentence, and no GPGLL is currently stored for parsing,
                // and no GPGLL parsing is already pending, then store this GPGLL sentence.
                if (is_gpgll && gpgll_storage_buf[0] == '\0' && !(ps->flags & PROG_FLAG_PARSED_GPGLL_PENDING)) {
                    strncpy(gpgll_storage_buf, current_sentence_content, gpgll_storage_size - 1);
                    gpgll_storage_buf[gpgll_storage_size - 1] = '\0'; // Ensure null termination.
                    ps->flags |= PROG_FLAG_PARSED_GPGLL_PENDING; // Mark that a GPGLL is ready for parsing.
                }
                // The line has been processed (either sent raw, or sent raw and stored if GPGLL).
                // Remove it from the assembly buffer.
                remove_line_from_gps_assembly_buffer(ps, total_line_len);
                return; // Exit function as this line is handled.
            } else {
                // UI module indicated it couldn't send (e.g., its TX buffer is busy).
                // Do not remove the line from the assembly buffer; it will be retried in the next loop.
                return; 
            }
        } else {
            // This case (total_line_len >= sizeof(raw_line_to_send)) should ideally not be hit
            // if buffer sizes are consistent throughout the application.
            // If it occurs, discard the line from assembly buffer to prevent blocking.
            remove_line_from_gps_assembly_buffer(ps, total_line_len);
            return;
        }
    }

    // This part is reached if not in DEBUG_MODE_PRINT_RAW_GPS,
    // or if raw send was attempted but the UI module could not handle it (returned false).
    // If the current sentence is GPGLL:
    if (is_gpgll) {
        // And if no GPGLL sentence is currently stored for parsing,
        // and no GPGLL parsing is already marked as pending:
        if (gpgll_storage_buf[0] == '\0' && !(ps->flags & PROG_FLAG_PARSED_GPGLL_PENDING)) {
            // Store the content of this GPGLL sentence for later parsing.
            strncpy(gpgll_storage_buf, current_sentence_content, gpgll_storage_size - 1);
            gpgll_storage_buf[gpgll_storage_size - 1] = '\0'; // Ensure null termination.
            ps->flags |= PROG_FLAG_PARSED_GPGLL_PENDING; // Mark that a GPGLL is ready for parsing.
        }
    }
    // The line has been processed (either ignored if not GPGLL, or stored if GPGLL and conditions met).
    // Remove it from the assembly buffer.
    remove_line_from_gps_assembly_buffer(ps, total_line_len);
}

/**
 * @brief Handles parsing of a stored GPGLL sentence and requests its display via the UI module.
 *
 * This function is called when the `PROG_FLAG_PARSED_GPGLL_PENDING` flag is set,
 * indicating that a GPGLL NMEA sentence is stored in `gpgll_to_parse_storage` and
 * is ready to be parsed and displayed.
 *
 * Operations:
 * 1. Returns immediately if `PROG_FLAG_PARSED_GPGLL_PENDING` is not set.
 * 2. Checks if `gpgll_to_parse_storage` actually contains a sentence.
 * 3. If a sentence is present, it calls `nmea_parse_gpgll_and_format()` to parse it
 *    and format the relevant information (Time, Longitude, Latitude) into `parsed_data_output`.
 * 4. If parsing and formatting are successful:
 *    a. It calls `ui_handle_parsed_data_transmission()` to request the UI module
 *       to display this formatted data. The UI module is responsible for managing
 *       the actual transmission and clearing the `PROG_FLAG_PARSED_GPGLL_PENDING` flag
 *       and `gpgll_to_parse_storage` upon successful initiation of the display/transmission.
 * 5. If parsing fails, it clears `PROG_FLAG_PARSED_GPGLL_PENDING` and `gpgll_to_parse_storage`
 *    to prevent retrying a bad parse.
 * 6. If the flag was set but the storage was empty (a potential logic error), it clears
 *    the flag to attempt recovery.
 *
 * @param[in,out] ps Pointer to the program state structure. The `flags` member may be cleared.
 * @param[in,out] gpgll_to_parse_storage Buffer containing the raw GPGLL NMEA sentence content
 *                                       to be parsed. This buffer's content might be cleared by
 *                                       the UI module if parsing and transmission are successful.
 */
static void handle_gpgll_parsing_and_request_display(prog_state_t *ps, char* gpgll_to_parse_storage) {
    // If no GPGLL sentence is marked as pending for parsing, do nothing.
    if (!(ps->flags & PROG_FLAG_PARSED_GPGLL_PENDING)) return;

    // The ui_handle_parsed_data_transmission function will internally check other
    // TX busy conditions (e.g., if its own buffer is full or platform USART is busy).

    // Check if there is actually a GPGLL sentence in the storage buffer.
    if (gpgll_to_parse_storage[0] != '\0') {
        // Buffer to hold the output string after parsing and formatting by nmea_parse_gpgll_and_format.
        // This output is typically "Time | Lon | Lat\r\n".
        char parsed_data_output[TX_BUFFER_SIZE_APP]; // Sized to accommodate the formatted output.

        // Attempt to parse the stored GPGLL sentence and format it into parsed_data_output.
        if (nmea_parse_gpgll_and_format(gpgll_to_parse_storage, parsed_data_output, sizeof(parsed_data_output))) {
            // Parsing and formatting were successful.
            // Request the UI module to handle the transmission of this parsed data.
            // The UI module will manage its own transmission buffer and related flags,
            // including clearing PROG_FLAG_PARSED_GPGLL_PENDING and gpgll_to_parse_storage
            // if it successfully initiates the transmission.
            ui_handle_parsed_data_transmission(ps, parsed_data_output, gpgll_to_parse_storage, DEBUG_MODE_PRINT_RAW_GPS);
        } else {
            // Parsing failed for the sentence in gpgll_to_parse_storage.
            // Clear the pending flag to avoid retrying this bad sentence.
            ps->flags &= ~PROG_FLAG_PARSED_GPGLL_PENDING;
            // Clear the storage buffer to remove the unparsable sentence.
            gpgll_to_parse_storage[0] = '\0';
            // Optional: Log this failure if a debugging mechanism is available.
            // DEBUG_PRINT("Main: Parsing stored GPGLL failed.");
        }
    } else {
        // PROG_FLAG_PARSED_GPGLL_PENDING was set, but the storage buffer is unexpectedly empty.
        // This indicates a potential logic error elsewhere in the application.
        // Clear the flag to attempt recovery and prevent a stall.
        ps->flags &= ~PROG_FLAG_PARSED_GPGLL_PENDING;
        // Optional: Log this error if a debugging mechanism is available.
        // DEBUG_PRINT("Main: Error: PARSED_GPGLL_PENDING set, but no GPGLL stored.");
    }
}

/**
 * @brief Executes one iteration of the main program loop.
 *
 * This function encapsulates the core logic that runs repeatedly. It handles:
 * 1. Turning off an activity LED (PA15) at the start of the loop.
 * 2. Processing platform events (like button presses).
 * 3. Handling GPS data reception (if not using fake data).
 * 4. Managing UI banner transmission requests.
 * 5. Processing received GPS sentences (if not using fake data for GPGLL).
 * 6. Injecting fake GPGLL data at intervals if `USE_FAKE_GPS_DATA` is true.
 * 7. Handling the parsing and display request for GPGLL data (real or fake).
 *
 * @param[in,out] ps Pointer to the program state structure, which is modified by
 *                   the various handler functions called within this loop.
 */
static void prog_loop_one(prog_state_t *ps)
{
    // Static buffer within main.c's scope to hold a raw GPGLL NMEA sentence
    // that has been identified from the GPS stream and is awaiting parsing.
    // Initialized to all zeros once at program start.
    static char gpgll_raw_nmea_to_parse[MAX_GPGLL_STORE_LEN_APP] = {0}; 
    
#if USE_FAKE_GPS_DATA
    // Static variables for managing fake data injection.
    static int fake_data_index = 0; // Index to cycle through fake_gpgll_sentences array.
    static int loop_counter_for_fake_data = 0; // Counter to time the injection of fake data.
    // Defines how many loop iterations to wait before injecting the next fake sentence.
    const int fake_data_interval = 5000; 
#endif

    // Turn off the activity LED (PA15) at the beginning of each loop iteration.
    // It will be turned on again if GPS data is received in handle_gps_reception.
    LED_ACTIVITY_PORT_GROUP.PORT_OUTCLR = LED_ACTIVITY_PIN;

    // Handle platform-level events (e.g., button presses for banner display).
	handle_platform_events(ps);
    
#if !USE_FAKE_GPS_DATA 
    // If not using fake GPS data, handle real GPS data reception.
    handle_gps_reception(ps);
#else
    // If USE_FAKE_GPS_DATA is true:
    // We might still want platform_do_loop_one() to run (handled in handle_platform_events).
    // The fake data injection logic below will take precedence for GPGLL sentences.
    // If real GPS data is also being received, this minimal handling keeps the async reception
    // chain alive but doesn't process its GPGLL content if we are faking.
    if (ps->gps_rx_desc.compl_type == PLATFORM_USART_RX_COMPL_DATA) {
        ps->gps_rx_desc.compl_type = PLATFORM_USART_RX_COMPL_NONE; // Reset completion flag.
        gps_platform_usart_cdc_rx_async(&ps->gps_rx_desc); // Re-arm GPS reception.
    }
#endif

    // --- UI and Data Transmission Handling ---
    // Handle transmission of the application banner if it's pending.
    ui_handle_banner_transmission(ps);
    
#if !USE_FAKE_GPS_DATA
    // If not using fake GPS data, process any assembled real GPS sentences.
    // This will identify GPGLL sentences and store them in gpgll_raw_nmea_to_parse.
    handle_gps_sentence_processing(ps, gpgll_raw_nmea_to_parse, sizeof(gpgll_raw_nmea_to_parse));
#else
    // If using fake data, the role of handle_gps_sentence_processing for GPGLL is bypassed
    // by the fake data injection logic below, which directly populates gpgll_raw_nmea_to_parse
    // and sets PROG_FLAG_PARSED_GPGLL_PENDING.
    // Non-GPGLL sentences from a real connected GPS might still be printed if DEBUG_MODE_PRINT_RAW_GPS
    // is true and handle_gps_sentence_processing is called (which it is not in this #else block).
    // Current logic: if faking, we don't call handle_gps_sentence_processing.
    // Any PROG_FLAG_GPS_UPDATE_PENDING from real GPS is implicitly ignored for GPGLL when faking.
#endif

#if USE_FAKE_GPS_DATA
    // Increment the counter for timing fake data injection.
    loop_counter_for_fake_data++;
    // Check if it's time to inject the next fake GPGLL sentence.
    if (loop_counter_for_fake_data >= fake_data_interval) {
        loop_counter_for_fake_data = 0; // Reset the counter.

        // Inject fake data only if:
        // 1. No GPGLL data (real or previously faked) is currently pending parsing.
        // 2. The main TX buffer (managed by UI/platform) is not busy.
        // 3. The underlying platform USART TX is not busy.
        // This prevents flooding the system or overwriting data that hasn't been processed.
        if (!(ps->flags & PROG_FLAG_PARSED_GPGLL_PENDING) && 
            !(ps->flags & PROG_FLAG_TX_BUFFER_BUSY) && 
            !platform_usart_cdc_tx_busy()) {

            // Copy the next fake GPGLL sentence into the parsing buffer.
            strncpy(gpgll_raw_nmea_to_parse, fake_gpgll_sentences[fake_data_index], MAX_GPGLL_STORE_LEN_APP - 1);
            gpgll_raw_nmea_to_parse[MAX_GPGLL_STORE_LEN_APP - 1] = '\0'; // Ensure null termination.
            
            // Set the flag to indicate that a GPGLL sentence is ready for parsing.
            ps->flags |= PROG_FLAG_PARSED_GPGLL_PENDING;

            // If DEBUG_MODE_PRINT_RAW_GPS is true, the raw fake GPGLL sentence will be printed
            // by the ui_handle_parsed_data_transmission function when it processes this fake data.
            // No need for a separate call to ui_handle_raw_data_transmission here for the fake GPGLL.
            
            // Advance to the next fake sentence in the array, wrapping around if necessary.
            fake_data_index = (fake_data_index + 1) % num_fake_sentences;
        }
    }
#endif

    // Handle the parsing and display request for the GPGLL sentence
    // currently in gpgll_raw_nmea_to_parse (whether it's real or fake).
    handle_gpgll_parsing_and_request_display(ps, gpgll_raw_nmea_to_parse);
}

/**
 * @brief Main entry point of the application.
 *
 * Initializes the program state and hardware, then enters an infinite loop
 * calling `prog_loop_one()` to perform the application's work.
 *
 * @return int This function should ideally not return. A return value of 1
 *             is provided as a fallback, though unreachable in normal operation.
 */
int main(void)
{
    // Declare an instance of the program state structure on the stack.
	prog_state_t ps_instance;
    // Perform initial setup of the program state and hardware.
	prog_setup(&ps_instance);

    // Enter the main application loop, which runs indefinitely.
	for (;;) {
        // Execute one iteration of the program's main logic.
		prog_loop_one(&ps_instance);
	}
    // This line should not be reached in a typical embedded application.
    return 1; 
}