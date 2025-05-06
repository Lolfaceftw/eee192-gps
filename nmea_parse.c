#include "nmea_parser.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h> // For atoi

// NMEA sentence specifics used internally by the parser
#define NMEA_GPGLL_PREFIX_INTERNAL "$GPGLL,"
#define NMEA_GPGLL_PREFIX_LEN_INTERNAL (sizeof(NMEA_GPGLL_PREFIX_INTERNAL) - 1)
#define NMEA_LINE_ENDING_INTERNAL "\r\n" // If sentences passed in might have it

// GPGLL Field Indices (relative to start of data after "$GPGLL,")
#define GPGLL_FIELD_LAT_VAL    0
#define GPGLL_FIELD_LAT_DIR    1
#define GPGLL_FIELD_LON_VAL    2
#define GPGLL_FIELD_LON_DIR    3
#define GPGLL_FIELD_UTC_TIME   4
#define GPGLL_MAX_FIELDS       7 // Lat, N/S, Lon, E/W, Time, Status, Mode

// Timezone offset for local time conversion (e.g., +8 for UTC+8)
#define LOCAL_TIMEZONE_OFFSET_HOURS 8

/**
 * @brief Helper function to parse UTC time string and format to local time.
 *
 * @param utc_time_str Raw UTC time string (e.g., "hhmmss.ss").
 * @param out_buf Buffer to store formatted local time "HH:MM:SS".
 * @param out_buf_size Size of out_buf.
 * @return true if successful, false otherwise.
 */
static bool format_local_time(const char* utc_time_str, char* out_buf, size_t out_buf_size) {
    if (!utc_time_str || strlen(utc_time_str) < 6) { // Need at least "hhmmss"
        snprintf(out_buf, out_buf_size, "--:--:--");
        return false;
    }

    char hh_str[3] = {0}, mm_str[3] = {0}, ss_str[3] = {0};
    strncpy(hh_str, utc_time_str, 2);
    strncpy(mm_str, utc_time_str + 2, 2);
    strncpy(ss_str, utc_time_str + 4, 2);

    int hour_utc = atoi(hh_str);
    int minute = atoi(mm_str);
    int second = atoi(ss_str);

    // Convert to local time by adding offset
    int hour_local = hour_utc + LOCAL_TIMEZONE_OFFSET_HOURS;

    // Handle day rollover
    if (hour_local >= 24) {
        hour_local -= 24;
        // Note: Date would change here, but GPGLL doesn't carry date.
    } else if (hour_local < 0) {
        hour_local += 24; // For negative offsets, though not used here.
    }

    snprintf(out_buf, out_buf_size, "%02d:%02d:%02d", hour_local, minute, second);
    return true;
}

bool nmea_parse_gpgll_and_format(const char* gpgll_sentence, char* out_buf, size_t out_buf_size) {
    if (gpgll_sentence == NULL || out_buf == NULL || out_buf_size == 0) {
        return false;
    }
    out_buf[0] = '\0'; // Initialize output buffer

    // Validate GPGLL prefix
    if (strncmp(gpgll_sentence, NMEA_GPGLL_PREFIX_INTERNAL, NMEA_GPGLL_PREFIX_LEN_INTERNAL) != 0) {
        return false; // Not a GPGLL sentence
    }

    // Create a mutable copy for tokenizing
    // NMEA_PARSER_MAX_GPGLL_CONTENT_LEN is for the content, add prefix len for full sentence.
    char temp_sentence[NMEA_PARSER_MAX_GPGLL_CONTENT_LEN + NMEA_GPGLL_PREFIX_LEN_INTERNAL + 1]; 
    strncpy(temp_sentence, gpgll_sentence, sizeof(temp_sentence) - 1);
    temp_sentence[sizeof(temp_sentence) - 1] = '\0';

    char* fields[GPGLL_MAX_FIELDS];
    for (int i = 0; i < GPGLL_MAX_FIELDS; ++i) fields[i] = ""; // Initialize with empty strings

    char *tokenizer_ptr = temp_sentence + NMEA_GPGLL_PREFIX_LEN_INTERNAL; // Start after "$GPGLL,"
    int field_count = 0;
    char *field_start = tokenizer_ptr;

    // Tokenize the sentence
    while (*tokenizer_ptr && field_count < GPGLL_MAX_FIELDS) {
        if (*tokenizer_ptr == ',' || *tokenizer_ptr == '*') { // Field delimiter or checksum start
            char delimiter = *tokenizer_ptr;
            *tokenizer_ptr = '\0'; // Null-terminate current field
            fields[field_count++] = field_start;
            field_start = tokenizer_ptr + 1;
            if (delimiter == '*') break; // Checksum reached, no more fields
        }
        tokenizer_ptr++;
    }
    // Capture the last field if it exists and wasn't followed by a delimiter (e.g., mode indicator)
    if (*field_start != '\0' && field_count < GPGLL_MAX_FIELDS && field_start < tokenizer_ptr) {
         fields[field_count++] = field_start;
    }

    const char* lat_val_str = fields[GPGLL_FIELD_LAT_VAL];
    const char* lat_dir_str = fields[GPGLL_FIELD_LAT_DIR];
    const char* lon_val_str = fields[GPGLL_FIELD_LON_VAL];
    const char* lon_dir_str = fields[GPGLL_FIELD_LON_DIR];
    const char* utc_time_str = (field_count > GPGLL_FIELD_UTC_TIME) ? fields[GPGLL_FIELD_UTC_TIME] : "";

    // --- Prepare Time Output String (Local Time) ---
    char time_output_str[NMEA_PARSER_MAX_TIME_STR_LEN];
    format_local_time(utc_time_str, time_output_str, sizeof(time_output_str));

    // --- Prepare Latitude Output String ---
    char lat_output_str[NMEA_PARSER_MAX_COORD_STR_LEN];
    bool lat_has_value = (strlen(lat_val_str) > 0);
    bool lat_has_direction = (strlen(lat_dir_str) > 0 && (lat_dir_str[0] == 'N' || lat_dir_str[0] == 'S'));

    if (lat_has_value && lat_has_direction) {
        snprintf(lat_output_str, sizeof(lat_output_str), "Lat: %s, %c", lat_val_str, lat_dir_str[0]);
    } else if (lat_has_value) {
        snprintf(lat_output_str, sizeof(lat_output_str), "Lat: %s", lat_val_str);
    } else {
        snprintf(lat_output_str, sizeof(lat_output_str), "Lat: Waiting...");
    }

    // --- Prepare Longitude Output String ---
    char lon_output_str[NMEA_PARSER_MAX_COORD_STR_LEN];
    bool lon_has_value = (strlen(lon_val_str) > 0);
    bool lon_has_direction = (strlen(lon_dir_str) > 0 && (lon_dir_str[0] == 'E' || lon_dir_str[0] == 'W'));

    if (lon_has_value && lon_has_direction) {
        snprintf(lon_output_str, sizeof(lon_output_str), "Long: %s, %c", lon_val_str, lon_dir_str[0]);
    } else if (lon_has_value) {
        snprintf(lon_output_str, sizeof(lon_output_str), "Long: %s", lon_val_str);
    } else {
        snprintf(lon_output_str, sizeof(lon_output_str), "Long: Waiting...");
    }

    // --- Combine into the final output buffer ---
    // Using NMEA_LINE_ENDING_INTERNAL for consistency if parser might handle line endings.
    // However, the output format is for display, so \r\n is standard.
    int written = snprintf(out_buf, out_buf_size, "%s | %s | %s%s",
                           time_output_str,
                           lon_output_str,
                           lat_output_str,
                           NMEA_LINE_ENDING_INTERNAL); // Or just "\r\n"

    return (written > 0 && (size_t)written < out_buf_size);
}