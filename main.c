/**
 * @file main.c
 * @brief Main application file for the GPS Data Logger and Parser.
 *
 * Orchestrates GPS data reception, NMEA sentence processing (via nmea_parser module),
 * and terminal output (via terminal_ui module). Manages overall application state.
 * The definition of the program state (prog_state_t) and its associated flags
 * are now located in main.h.
 *
 * @author Alberto de Villa <alberto.de.villa@eee.upd.edu.ph> (Original)
 * @author Estrada (Supplemented by EEE 158 AY 24-25 1S)
 * @author Christian Klein C. Ramos (Supplemented by EEE 192 AY 24-25 2S, Refactored, Modularized)
 * @date May 6, 2025 // Last significant modification date
 */

// Standard C and Microcontroller specific includes
#include <xc.h>
#include <string.h>
#include <stdio.h>     // For snprintf (used by nmea_parser and terminal_ui)
#include <stdbool.h>
#include <stdlib.h>

// Project-specific includes
#include "main.h"          // For prog_state_t, program flags, and core app constants
#include "platform.h"
#include "nmea_parser.h"
#include "terminal_ui.h"   // For UI handling functions

// --- Application Configuration ---
// This constant is used by main logic and passed to UI functions.
static const bool DEBUG_MODE_PRINT_RAW_GPS = true;

// --- Application Constants (main.c specific, if any beyond main.h) ---
// ANSI codes and banner_msg are now managed by terminal_ui.c
// Buffer sizes and prog_state_t flags are in main.h
// NMEA identification prefixes are specific to main.c's pre-filtering logic.
#define APP_NMEA_LINE_ENDING "\r\n"
#define APP_NMEA_GPGLL_PREFIX "$GPGLL,"
#define APP_NMEA_GPGLL_PREFIX_LEN (sizeof(APP_NMEA_GPGLL_PREFIX) - 1)

// LED Indicator (specific to main.c's direct hardware interaction)
#define LED_ACTIVITY_PORT_GROUP (PORT_SEC_REGS->GROUP[0])
#define LED_ACTIVITY_PIN        (1 << 15)

// prog_state_t typedef and PROG_FLAG_* definitions are in main.h

// --- FAKE GPS DATA FOR DEBUGGING ---
#define USE_FAKE_GPS_DATA false // Set to true to use fake data, false for real GPS
                               // IMPORTANT: If true, real GPS data processing for GPGLL will be overridden

#if USE_FAKE_GPS_DATA
// Array of fake GPGLL sentences to cycle through
static const char* fake_gpgll_sentences[] = {
    "$GPGLL,4043.9620,N,07959.0350,W,235959.00,A,A*77", // Example: Pittsburgh, PA, USA (Night)
    "$GPGLL,3403.7658,S,15052.9787,E,123045.10,A,A*6C", // Example: Sydney, Australia (Day)
    "$GPGLL,4807.038,N,01131.000,E,104820.22,A,A*4D",   // Example: Munich, Germany
    "$GPGLL,2237.0000,N,11408.0000,E,081530.00,A,A*7A", // Example: Hong Kong
    "$GPGLL,,,,,123519.00,V,N*4D",                     // Example: Invalid/No Fix, time only
    "$GPGLL,5130.0000,N,00007.0000,W,140000.00,A,A*78"  // Example: Greenwich, London
};
static const int num_fake_sentences = sizeof(fake_gpgll_sentences) / sizeof(fake_gpgll_sentences[0]);
#endif
// --- End FAKE GPS DATA ---

// --- Static Function Prototypes (main.c internal logic) ---
static void handle_platform_events(prog_state_t *ps);
static void handle_gps_reception(prog_state_t *ps);
static void handle_gps_sentence_processing(prog_state_t *ps, char* gpgll_storage_buf, size_t gpgll_storage_size);
static void handle_gpgll_parsing_and_request_display(prog_state_t *ps, char* gpgll_to_parse_storage);
static void remove_line_from_gps_assembly_buffer(prog_state_t *ps, int line_len_with_crlf);

static void prog_setup(prog_state_t *ps)
{
	memset(ps, 0, sizeof(prog_state_t));
    ps->banner_has_been_displayed_this_session = false;
	platform_init();
	
	ps->cdc_rx_desc.buf     = ps->cdc_rx_buf;
	ps->cdc_rx_desc.max_len = RX_BUFFER_CDC_SIZE_APP; // Constant from main.h
	platform_usart_cdc_rx_async(&ps->cdc_rx_desc);
    
    ps->gps_rx_desc.buf     = ps->gps_rx_buf;
    ps->gps_rx_desc.max_len = RX_BUFFER_GPS_SIZE_APP; // Constant from main.h
    gps_platform_usart_cdc_rx_async(&ps->gps_rx_desc);
}

static void handle_platform_events(prog_state_t *ps) {
    platform_do_loop_one();
    uint16_t pb_event = platform_pb_get_event();
	if ((pb_event & PLATFORM_PB_ONBOARD_PRESS) != 0) {
		ps->flags |= PROG_FLAG_BANNER_PENDING; // PROG_FLAG_* defined in main.h
	}
}

static void handle_gps_reception(prog_state_t *ps) {
    if (ps->gps_rx_desc.compl_type == PLATFORM_USART_RX_COMPL_DATA) {
        LED_ACTIVITY_PORT_GROUP.PORT_OUTSET = LED_ACTIVITY_PIN;
        uint16_t received_len = ps->gps_rx_desc.compl_info.data_len;

        if (ps->gps_assembly_len + received_len < GPS_ASSEMBLY_BUF_SIZE_APP) { // Constant from main.h
            memcpy(ps->gps_assembly_buf + ps->gps_assembly_len, ps->gps_rx_buf, received_len);
            ps->gps_assembly_len += received_len;
            ps->gps_assembly_buf[ps->gps_assembly_len] = '\0';

            if (strstr(ps->gps_assembly_buf, APP_NMEA_LINE_ENDING) != NULL) {
                if (!(ps->flags & PROG_FLAG_GPS_UPDATE_PENDING)) {
                     ps->flags |= PROG_FLAG_GPS_UPDATE_PENDING;
                }
            }
        } else {
            ps->gps_assembly_len = 0;
            ps->gps_assembly_buf[0] = '\0';
        }
        ps->gps_rx_desc.compl_type = PLATFORM_USART_RX_COMPL_NONE;
        gps_platform_usart_cdc_rx_async(&ps->gps_rx_desc);
    }
}

static void remove_line_from_gps_assembly_buffer(prog_state_t *ps, int line_len_with_crlf) {
    if (line_len_with_crlf <= 0 || (uint16_t)line_len_with_crlf > ps->gps_assembly_len) {
        ps->gps_assembly_len = 0;
        ps->gps_assembly_buf[0] = '\0';
        ps->flags &= ~PROG_FLAG_GPS_UPDATE_PENDING;
        return;
    }
    ps->gps_assembly_len -= line_len_with_crlf;
    memmove(ps->gps_assembly_buf, ps->gps_assembly_buf + line_len_with_crlf, ps->gps_assembly_len);
    ps->gps_assembly_buf[ps->gps_assembly_len] = '\0';

    if (strstr(ps->gps_assembly_buf, APP_NMEA_LINE_ENDING) == NULL) {
        ps->flags &= ~PROG_FLAG_GPS_UPDATE_PENDING;
    }
}

static void handle_gps_sentence_processing(prog_state_t *ps, char* gpgll_storage_buf, size_t gpgll_storage_size) {
    if (!(ps->flags & PROG_FLAG_GPS_UPDATE_PENDING)) return;
    // The UI functions called below will check PROG_FLAG_TX_BUFFER_BUSY and platform_usart_cdc_tx_busy()
    // No need to check them here before calling UI functions.

    char* newline_ptr = strstr(ps->gps_assembly_buf, APP_NMEA_LINE_ENDING);
    if (newline_ptr == NULL) {
        ps->flags &= ~PROG_FLAG_GPS_UPDATE_PENDING;
        return;
    }

    int sentence_content_len = newline_ptr - ps->gps_assembly_buf;
    int total_line_len = sentence_content_len + strlen(APP_NMEA_LINE_ENDING);

    char current_sentence_content[GPS_ASSEMBLY_BUF_SIZE_APP];
    if (sentence_content_len >= (int)sizeof(current_sentence_content)) {
        remove_line_from_gps_assembly_buffer(ps, total_line_len);
        return;
    }
    memcpy(current_sentence_content, ps->gps_assembly_buf, sentence_content_len);
    current_sentence_content[sentence_content_len] = '\0';

    bool is_gpgll = (strncmp(current_sentence_content, APP_NMEA_GPGLL_PREFIX, APP_NMEA_GPGLL_PREFIX_LEN) == 0);

    if (DEBUG_MODE_PRINT_RAW_GPS) {
        // Prepare the raw line from the assembly buffer to pass to the UI function.
        // Ensure the buffer is large enough. total_line_len includes CRLF.
        char raw_line_to_send[GPS_ASSEMBLY_BUF_SIZE_APP]; // Max possible line size
        if(total_line_len < sizeof(raw_line_to_send)){
             memcpy(raw_line_to_send, ps->gps_assembly_buf, total_line_len);
            // raw_line_to_send[total_line_len] = '\0'; // Not strictly needed as ui_handle_raw_data_transmission takes length.

            if (ui_handle_raw_data_transmission(ps, raw_line_to_send, total_line_len)) {
                // Raw send initiated successfully by UI module.
                if (is_gpgll && gpgll_storage_buf[0] == '\0' && !(ps->flags & PROG_FLAG_PARSED_GPGLL_PENDING)) {
                    strncpy(gpgll_storage_buf, current_sentence_content, gpgll_storage_size - 1);
                    gpgll_storage_buf[gpgll_storage_size - 1] = '\0';
                    ps->flags |= PROG_FLAG_PARSED_GPGLL_PENDING;
                }
                remove_line_from_gps_assembly_buffer(ps, total_line_len);
                return; // Line processed
            } else {
                // UI module indicated it couldn't send (e.g., TX busy). Do not remove line.
                return; 
            }
        } else {
             // This case should ideally not be hit if buffer sizes are consistent.
            remove_line_from_gps_assembly_buffer(ps, total_line_len); // Discard if too long for temp buffer
            return;
        }
    }

    // If not in DEBUG_MODE_PRINT_RAW_GPS or if raw send was skipped/failed
    if (is_gpgll) {
        if (gpgll_storage_buf[0] == '\0' && !(ps->flags & PROG_FLAG_PARSED_GPGLL_PENDING)) {
            strncpy(gpgll_storage_buf, current_sentence_content, gpgll_storage_size - 1);
            gpgll_storage_buf[gpgll_storage_size - 1] = '\0';
            ps->flags |= PROG_FLAG_PARSED_GPGLL_PENDING;
        }
    }
    remove_line_from_gps_assembly_buffer(ps, total_line_len); // Line processed or stored
}

static void handle_gpgll_parsing_and_request_display(prog_state_t *ps, char* gpgll_to_parse_storage) {
    if (!(ps->flags & PROG_FLAG_PARSED_GPGLL_PENDING)) return;
    // The ui_handle_parsed_data_transmission function will check other TX busy conditions.

    if (gpgll_to_parse_storage[0] != '\0') {
        // Buffer for the output of nmea_parse_gpgll_and_format (the "Time | Lon | Lat\r\n" string)
        char parsed_data_output[TX_BUFFER_SIZE_APP]; // Should be large enough for formatted output

        if (nmea_parse_gpgll_and_format(gpgll_to_parse_storage, parsed_data_output, sizeof(parsed_data_output))) {
            // Parsing successful. Request UI module to display it.
            // The UI module will handle its transmission and related flag updates (TX_BUFFER_BUSY, PARSED_GPGLL_PENDING).
            ui_handle_parsed_data_transmission(ps, parsed_data_output, gpgll_to_parse_storage, DEBUG_MODE_PRINT_RAW_GPS);
        } else {
            // Parsing failed. Clear flags and storage to prevent retrying a bad parse.
            ps->flags &= ~PROG_FLAG_PARSED_GPGLL_PENDING;
            gpgll_to_parse_storage[0] = '\0';
            // Consider logging: DEBUG_PRINT("Main: Parsing stored GPGLL failed.");
        }
    } else {
        // PROG_FLAG_PARSED_GPGLL_PENDING was set, but storage is empty. Logic error.
        ps->flags &= ~PROG_FLAG_PARSED_GPGLL_PENDING; // Clear flag to recover.
        // Consider logging: DEBUG_PRINT("Main: Error: PARSED_GPGLL_PENDING set, but no GPGLL stored.");
    }
}

static void prog_loop_one(prog_state_t *ps)
{
    // Static buffer in main.c to hold the raw GPGLL NMEA sentence content before parsing.
    static char gpgll_raw_nmea_to_parse[MAX_GPGLL_STORE_LEN_APP] = {0}; 
    
#if USE_FAKE_GPS_DATA
    static int fake_data_index = 0;
    static int loop_counter_for_fake_data = 0;
    const int fake_data_interval = 5000; // Inject fake data every ~5000 loop iterations
#endif

    LED_ACTIVITY_PORT_GROUP.PORT_OUTCLR = LED_ACTIVITY_PIN;

	handle_platform_events(ps);
    
#if !USE_FAKE_GPS_DATA // Only handle real GPS reception if not using fake data for GPGLL
    handle_gps_reception(ps);
#else
    // If using fake data, we might still want to call platform_do_loop_one()
    // and potentially clear any pending real GPS flags to avoid interference,
    // or ensure the fake data injection below always takes precedence.
    // For simplicity, if USE_FAKE_GPS_DATA is true, we'll just let the fake data
    // overwrite gpgll_raw_nmea_to_parse.
    // You could optionally disable actual GPS UART reception here if needed
    // to save power or prevent buffer overflows if the GPS module is still sending.
    // For now, we assume handle_gps_reception might still fill buffers, but
    // handle_gps_sentence_processing will be affected by the fake data logic.
    if (ps->gps_rx_desc.compl_type == PLATFORM_USART_RX_COMPL_DATA) {
        // Minimal handling to keep GPS async reception going if it was started
        // but we won't process its content for GPGLL if faking.
        ps->gps_rx_desc.compl_type = PLATFORM_USART_RX_COMPL_NONE;
        gps_platform_usart_cdc_rx_async(&ps->gps_rx_desc);
    }
#endif

    // --- UI and Data Transmission Handling ---
    ui_handle_banner_transmission(ps);
    
#if !USE_FAKE_GPS_DATA
    handle_gps_sentence_processing(ps, gpgll_raw_nmea_to_parse, sizeof(gpgll_raw_nmea_to_parse));
#else
    // If using fake data, we control the PROG_FLAG_PARSED_GPGLL_PENDING flag
    // and gpgll_raw_nmea_to_parse directly.
    // We can effectively bypass handle_gps_sentence_processing's role for GPGLL.
    // However, if DEBUG_MODE_PRINT_RAW_GPS is on and handle_gps_sentence_processing
    // prints ALL raw sentences, you might see real non-GPGLL sentences if GPS is connected.
    // This is usually fine for debugging.
    // Let's ensure any flags set by real GPS processing are cleared if we are about to inject fake data.
    if (ps->flags & PROG_FLAG_GPS_UPDATE_PENDING) {
         // If real GPS data came in, but we are faking, clear its pending status
         // or just let the fake data overwrite. For now, let it overwrite.
    }
#endif

#if USE_FAKE_GPS_DATA
    loop_counter_for_fake_data++;
    if (loop_counter_for_fake_data >= fake_data_interval) {
        loop_counter_for_fake_data = 0;

        // Check if the previous fake/parsed data has been handled (flag cleared)
        // or if the TX buffer is free, to avoid flooding or overwriting too quickly.
        // This check helps ensure one piece of fake data is processed before injecting the next.
        if (!(ps->flags & PROG_FLAG_PARSED_GPGLL_PENDING) && 
            !(ps->flags & PROG_FLAG_TX_BUFFER_BUSY) && 
            !platform_usart_cdc_tx_busy()) {

            strncpy(gpgll_raw_nmea_to_parse, fake_gpgll_sentences[fake_data_index], MAX_GPGLL_STORE_LEN_APP - 1);
            gpgll_raw_nmea_to_parse[MAX_GPGLL_STORE_LEN_APP - 1] = '\0'; // Ensure null termination
            
            ps->flags |= PROG_FLAG_PARSED_GPGLL_PENDING; // Signal that there's GPGLL data to parse

            // Optionally, if DEBUG_MODE_PRINT_RAW_GPS is true, and you want to simulate the raw print
            // separately for the fake data (though ui_handle_parsed_data_transmission might do it):
            if (DEBUG_MODE_PRINT_RAW_GPS) {
                // The ui_handle_parsed_data_transmission will receive this fake sentence
                // and if DEBUG_MODE_PRINT_RAW_GPS is true, it should print it.
                // No need to explicitly call ui_handle_raw_data_transmission here for the fake GPGLL
                // as the existing flow for parsed data should cover it.
            }
            
            fake_data_index = (fake_data_index + 1) % num_fake_sentences; // Move to next fake sentence
        }
    }
#endif

    handle_gpgll_parsing_and_request_display(ps, gpgll_raw_nmea_to_parse);
}
int main(void)
{
	prog_state_t ps_instance;
	prog_setup(&ps_instance);
	for (;;) {
		prog_loop_one(&ps_instance);
	}
    return 1; // Should not be reached
}