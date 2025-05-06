#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <stdbool.h>
#include <stddef.h> // For size_t

// --- NMEA Constants ---
// (Consider moving NMEA-specific prefixes here if they are primarily for the parser's use)
// For now, main.c uses NMEA_GPGLL_PREFIX for pre-filtering, so it can stay there or be duplicated.
// If the parser were more generic, it might define these internally or take sentence type as an argument.

// Maximum length for a GPGLL sentence content (excluding $GPGLL, and CRLF)
// This should be coordinated with buffer sizes used in the parser.
#define NMEA_PARSER_MAX_GPGLL_CONTENT_LEN 100 
// Maximum length for the formatted time string (e.g., "HH:MM:SS")
#define NMEA_PARSER_MAX_TIME_STR_LEN 12
// Maximum length for formatted lat/lon strings (e.g., "Lat: DDDMM.MMMM, C")
#define NMEA_PARSER_MAX_COORD_STR_LEN 64


/**
 * @brief Parses a GPGLL NMEA sentence and formats Time, Latitude, and Longitude into a buffer.
 *
 * Extracts UTC time, latitude, longitude, and their respective hemispheres from a GPGLL
 * sentence. Formats them into a human-readable string: "HH:MM:SS (Local) | Long: ... | Lat: ..."
 * The time is converted to a local timezone (e.g., UTC+8).
 *
 * @param gpgll_sentence The NMEA GPGLL sentence string (expected to start with "$GPGLL,").
 *                       The sentence should NOT include the leading "$GPGLL," prefix itself if
 *                       NMEA_GPGLL_PREFIX_LEN is used to skip it before calling,
 *                       OR it should include it and the function handles it.
 *                       Current implementation expects the full sentence including "$GPGLL,".
 * @param out_buf The buffer to write the formatted string into.
 * @param out_buf_size The size of the output buffer.
 * @return true if parsing and formatting were successful and fit in out_buf, false otherwise.
 */
bool nmea_parse_gpgll_and_format(const char* gpgll_sentence, char* out_buf, size_t out_buf_size);

#endif // NMEA_PARSER_H