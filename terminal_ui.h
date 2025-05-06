#ifndef TERMINAL_UI_H
#define TERMINAL_UI_H

#include <stdbool.h>
#include <stddef.h> // For size_t

// Forward declaration of prog_state_t to avoid circular dependencies with main.c
// The UI functions will operate on the main program's state, particularly its transmit buffer and flags.
struct prog_state_type;

/**
 * @brief Handles the transmission of the pre-defined banner message.
 *
 * If a banner display is pending (indicated by PROG_FLAG_BANNER_PENDING in ps->flags),
 * the USART transmitter is free, and the main TX buffer is not busy, this function
 * prepares and attempts to send the banner. It updates relevant flags in ps on success or failure.
 * It also sets ps->banner_has_been_displayed_this_session to true on successful queuing.
 *
 * @param ps Pointer to the main program_state_t structure.
 */
void ui_handle_banner_transmission(struct prog_state_type *ps);

/**
 * @brief Handles the formatting and transmission of parsed GPGLL data.
 *
 * If parsed GPGLL data is ready to be sent (indicated by PROG_FLAG_PARSED_GPGLL_PENDING
 * and a non-empty gpgll_to_parse string), the transmitter is free, and the main TX buffer
 * is not busy, this function:
 * 1. Formats the `parsed_data_str` with appropriate ANSI cursor positioning codes
 *    (behavior depends on DEBUG_MODE_PRINT_RAW_GPS and ps->banner_has_been_displayed_this_session).
 * 2. Populates ps->tx_buf with the final string.
 * 3. Attempts to send the data asynchronously using ps->tx_desc.
 * 4. Updates relevant flags in ps and clears the source `gpgll_to_parse` string on success.
 *
 * @param ps Pointer to the main program_state_t structure.
 * @param parsed_data_str Null-terminated string containing the already parsed GPGLL data
 *                        (e.g., "Time | Long | Lat\r\n").
 * @param gpgll_to_parse_storage Pointer to the buffer in main.c holding the raw GPGLL NMEA sentence
 *                               that was parsed. This will be cleared (first char set to '\0')
 *                               by this function if the transmission is successfully queued.
 * @param debug_mode_raw_gps The current state of DEBUG_MODE_PRINT_RAW_GPS.
 */
void ui_handle_parsed_data_transmission(struct prog_state_type *ps,
                                        const char *parsed_data_str,
                                        char* gpgll_to_parse_storage,
                                        bool debug_mode_raw_gps);

/**
 * @brief Handles the transmission of raw NMEA data (typically for debugging).
 *
 * If the transmitter is free and the main TX buffer is not busy, this function:
 * 1. Copies `raw_data_str` to ps->tx_buf.
 * 2. Attempts to send the data asynchronously using ps->tx_desc.
 * 3. Updates relevant flags in ps.
 *
 * @param ps Pointer to the main program_state_t structure.
 * @param raw_data_str Pointer to the raw NMEA data string (including CRLF).
 * @param raw_data_len Length of the raw_data_str.
 * @return true if the transmission was successfully initiated, false otherwise (e.g., TX busy, buffer full).
 *         Note: This return is for immediate feedback; main TX flags in 'ps' are the primary state.
 */
bool ui_handle_raw_data_transmission(struct prog_state_type *ps,
                                     const char *raw_data_str,
                                     size_t raw_data_len);

#endif // TERMINAL_UI_H