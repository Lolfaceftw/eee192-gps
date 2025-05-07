#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <stdbool.h>
#include <stddef.h> // For size_t

// --- NMEA Constants ---
// Maximum length for a GPGLL sentence content (excluding $GPGLL, and CRLF)
#define NMEA_PARSER_MAX_GPGLL_CONTENT_LEN 100
// Maximum length for the formatted time string (e.g., "HH:MM:SS")
#define NMEA_PARSER_MAX_TIME_STR_LEN 12
// Maximum length for formatted lat/lon strings (e.g., "Lat: DDD.DDDD, C" or "Lat: DDDMM.MMMM, C")
#define NMEA_PARSER_MAX_COORD_STR_LEN 64 // Increased slightly to accommodate potentially longer decimal degree strings

// --- NMEA Parser Configuration ---
/**
 * @brief If true, nmea_parse_gpgll_and_format will use predefined fake data
 *        instead of parsing the input sentence. Useful for testing formatting.
 */
static const bool NMEA_PARSER_USE_FAKE_GPGLL_DATA = true; // Set to true to enable fake data

/**
 * @brief Number of decimal places to use when formatting decimal degrees.
 */
#define NMEA_PARSER_DECIMAL_DEG_PRECISION 4

/**
 * @brief Parses a GPGLL NMEA sentence and formats Time, Latitude, and Longitude into a buffer.
 *
 * Extracts UTC time, latitude, longitude, and their respective hemispheres from a GPGLL
 * sentence. Formats them into a human-readable string: "HH:MM:SS (Local) | Lat: ... | Long: ..."
 * The time is converted to a local timezone (e.g., UTC+8).
 * Latitude and Longitude are converted to decimal degrees.
 *
 * @param gpgll_sentence The NMEA GPGLL sentence string (expected to start with "$GPGLL,").
 * @param out_buf The buffer to write the formatted string into.
 * @param out_buf_size The size of the output buffer.
 * @return true if parsing and formatting were successful and fit in out_buf, false otherwise.
 */
bool nmea_parse_gpgll_and_format(const char* gpgll_sentence, char* out_buf, size_t out_buf_size);

#endif // NMEA_PARSER_H