/**
 * @file      nmea_parser.c
 * @brief     Implementation of NMEA sentence parsing functions.
 *
 * This file contains the logic for parsing NMEA GPGLL sentences to extract
 * time, latitude, and longitude information. It includes helper functions for
 * time formatting (UTC to local) and coordinate conversion (NMEA format to decimal degrees).
 * The primary public function is `nmea_parse_gpgll_and_format`, which takes a raw
 * GPGLL sentence and produces a human-readable formatted string.
 *
 * @author    Alberto de Villa <alberto.de.villa@eee.upd.edu.ph> (Original Structure)
 * @author    Christian Klein C. Ramos (Docstrings, Comments, Adherence to Project Guidelines)
 * @date      07 May 2025
 */

#include "nmea_parser.h" // Corresponding header file for this module
#include <string.h>      // For string manipulation functions (strlen, strncmp, strncpy, strtok_r - though strtok_r is not used here, manual tokenizing is)
#include <stdio.h>       // For snprintf, used to format output strings
#include <stdlib.h>      // For atof (convert string to double) and atoi (convert string to integer)

// No need for math.h if fabs is not used, and convert_nmea_coord_to_degrees returns positive values.

// NMEA sentence specifics used internally by the parser
/** @brief Internal definition of the GPGLL NMEA sentence prefix. */
#define NMEA_GPGLL_PREFIX_INTERNAL "$GPGLL,"
/** @brief Length of the internal GPGLL NMEA sentence prefix, excluding null terminator. */
#define NMEA_GPGLL_PREFIX_LEN_INTERNAL (sizeof(NMEA_GPGLL_PREFIX_INTERNAL) - 1)
/** @brief Internal definition of the NMEA sentence line ending (CRLF).
 *         Used if sentences passed into the parser might still contain it,
 *         and for appending to the formatted output.
 */
#define NMEA_LINE_ENDING_INTERNAL "\r\n"

// GPGLL Field Indices (relative to start of data after "$GPGLL,")
// These indices define the expected position of each piece of information
// within the comma-separated fields of a GPGLL sentence.
#define GPGLL_FIELD_LAT_VAL    0 /**< Index for Latitude value field. */
#define GPGLL_FIELD_LAT_DIR    1 /**< Index for Latitude direction (N/S) field. */
#define GPGLL_FIELD_LON_VAL    2 /**< Index for Longitude value field. */
#define GPGLL_FIELD_LON_DIR    3 /**< Index for Longitude direction (E/W) field. */
#define GPGLL_FIELD_UTC_TIME   4 /**< Index for UTC time field. */
#define GPGLL_MAX_FIELDS       7 /**< Maximum expected number of fields in a GPGLL sentence (Lat, N/S, Lon, E/W, Time, Status, Mode). */

// Timezone offset for local time conversion (e.g., +8 for UTC+8)
/** @brief Timezone offset in hours to convert UTC time to local time.
 *         Positive for timezones ahead of UTC, negative for behind.
 *         Example: 8 for UTC+8 (e.g., PST, HKT).
 */
#define LOCAL_TIMEZONE_OFFSET_HOURS 8

/**
 * @brief Helper function to parse a UTC time string (hhmmss.ss) and format it to local time (HH:MM:SS).
 *
 * This function takes a raw UTC time string from an NMEA sentence, extracts the
 * hours, minutes, and seconds, applies a predefined `LOCAL_TIMEZONE_OFFSET_HOURS`,
 * and formats the result into a "HH:MM:SS" string. It handles day rollovers
 * (e.g., if UTC time + offset exceeds 23 hours).
 *
 * @param[in]  utc_time_str Pointer to the null-terminated string containing the UTC time
 *                          (e.g., "235959.00"). Expected format is at least "hhmmss".
 * @param[out] out_buf      Pointer to the character buffer where the formatted local time
 *                          string ("HH:MM:SS") will be stored.
 * @param[in]  out_buf_size The size of the `out_buf` buffer, including space for the
 *                          null terminator.
 * @return     `true` if the UTC time string was successfully parsed and formatted.
 * @return     `false` if `utc_time_str` is NULL, too short, or if formatting otherwise fails,
 *             in which case `out_buf` will contain "--:--:--".
 */
static bool format_local_time(const char* utc_time_str, char* out_buf, size_t out_buf_size) {
    // Validate input: check for NULL pointer or if the string is too short for "hhmmss".
    if (!utc_time_str || strlen(utc_time_str) < 6) {
        // If input is invalid, format output as "--:--:--" and return false.
        snprintf(out_buf, out_buf_size, "--:--:--");
        return false;
    }

    // Temporary buffers to hold parts of the UTC time string.
    char hh_str[3] = {0}; // For hours (hh)
    char mm_str[3] = {0}; // For minutes (mm)
    char ss_str[3] = {0}; // For seconds (ss)

    // Copy hour, minute, and second parts from the UTC time string.
    strncpy(hh_str, utc_time_str, 2);       // First 2 chars are hours.
    strncpy(mm_str, utc_time_str + 2, 2);   // Next 2 chars are minutes.
    strncpy(ss_str, utc_time_str + 4, 2);   // Next 2 chars are seconds.
    // Note: hh_str, mm_str, ss_str are already null-terminated due to array initialization.

    // Convert string parts to integers.
    int hour_utc = atoi(hh_str);
    int minute = atoi(mm_str);
    int second = atoi(ss_str);

    // Convert UTC hour to local time by adding the defined timezone offset.
    int hour_local = hour_utc + LOCAL_TIMEZONE_OFFSET_HOURS;

    // Handle day rollover if local hour goes beyond 23 or below 0.
    if (hour_local >= 24) {
        hour_local -= 24; // Adjust for next day.
        // Note: The date would change here, but GPGLL sentences do not carry date information.
        //       This function only handles time adjustment.
    } else if (hour_local < 0) {
        hour_local += 24; // Adjust for previous day (for negative timezone offsets, though not used with current positive offset).
    }

    // Format the calculated local time into the output buffer as "HH:MM:SS".
    snprintf(out_buf, out_buf_size, "%02d:%02d:%02d", hour_local, minute, second);
    return true; // Indicate successful formatting.
}

/**
 * @brief Converts an NMEA coordinate string (DDmm.mmmm or DDDmm.mmmm) to decimal degrees.
 *
 * NMEA coordinates are typically given in a format where the first few digits
 * represent whole degrees, and the subsequent digits represent minutes and
 * decimal fractions of minutes. This function parses such a string and converts
 * it into a single floating-point value representing decimal degrees.
 * For example, "4043.9620" (latitude) with `deg_len=2` becomes 40 + (43.9620 / 60.0).
 * "07959.0350" (longitude) with `deg_len=3` becomes 79 + (59.0350 / 60.0).
 * The function always returns a positive value; the direction (N/S, E/W) is handled separately.
 *
 * @param[in] value_str Pointer to the null-terminated NMEA coordinate string
 *                      (e.g., "4043.9620" for latitude, "07959.0350" for longitude).
 * @param[in] deg_len   The number of characters at the beginning of `value_str`
 *                      that represent the whole degrees part (e.g., 2 for latitude DD,
 *                      3 for longitude DDD).
 * @return    The coordinate converted to decimal degrees (always a positive value).
 *            Returns 0.0 if `value_str` is NULL, empty, or too short to extract degrees.
 */
static double convert_nmea_coord_to_degrees(const char* value_str, int deg_len) {
    // Validate input: check for NULL pointer or empty string.
    if (value_str == NULL || strlen(value_str) == 0) {
        return 0.0; // Return 0.0 for empty or invalid input.
    }
    
    // Check if the string is long enough to contain the degree part.
    if (strlen(value_str) < (size_t)deg_len) {
        return 0.0; // Not enough characters for the specified degree length.
    }

    // Temporary buffers for degree and minute parts of the coordinate string.
    // Sized to be safe for typical NMEA coordinate field lengths.
    char deg_str[8];  // Buffer for degrees part (e.g., "40" or "079").
    char min_str[16]; // Buffer for minutes part (e.g., "43.9620" or "59.0350").

    // Extract the degree part from the beginning of value_str.
    strncpy(deg_str, value_str, deg_len);
    deg_str[deg_len] = '\0'; // Null-terminate the degree string.
    // Convert the degree string to a double.
    double degrees = atof(deg_str);

    // Check if there's a minutes part remaining in value_str.
    if (strlen(value_str) > (size_t)deg_len) {
        // If yes, copy the rest of value_str (after the degree part) into min_str.
        strncpy(min_str, value_str + deg_len, sizeof(min_str) - 1);
        min_str[sizeof(min_str) - 1] = '\0'; // Ensure null termination.
    } else {
        // No minutes part present after the degrees.
        min_str[0] = '\0'; // Set min_str to an empty string.
    }
    // Convert the minute string (which might be empty) to a double. atof("") is 0.0.
    double minutes = atof(min_str);

    // Calculate decimal degrees: degrees + (minutes / 60.0).
    return degrees + (minutes / 60.0);
}

/**
 * @brief Parses a GPGLL NMEA sentence and formats extracted data into a human-readable string.
 *
 * This is the primary public function of the NMEA parser module. It takes a raw
 * GPGLL sentence string, tokenizes it to extract fields for latitude, longitude,
 * and UTC time. It then uses helper functions to convert coordinates to decimal
 * degrees and UTC time to local time. Finally, it formats this information into
 * a single output string: "LocalTime | Lat: DD.dddddd deg, N/S | Long: DDD.dddddd deg, E/W\r\n".
 *
 * If the input sentence is not a valid GPGLL sentence, or if essential fields are
 * missing or malformed, placeholder strings like "--:--:--" for time or
 * "Lat: Waiting for data..., -" for coordinates may be used in the output.
 *
 * @param[in]  gpgll_sentence Pointer to the null-terminated string containing the raw GPGLL NMEA sentence
 *                            (e.g., "$GPGLL,4043.9620,N,07959.0350,W,235959.00,A,A*77").
 * @param[out] out_buf        Pointer to the character buffer where the formatted output string will be stored.
 * @param[in]  out_buf_size   The size of the `out_buf` buffer, including space for the null terminator.
 * @return     `true` if the GPGLL sentence was successfully parsed and the output string was formatted
 *             without truncation and written to `out_buf`.
 * @return     `false` if input parameters are invalid, the sentence is not GPGLL, or if `snprintf`
 *             fails or indicates truncation. `out_buf` might be empty or contain partial data on failure.
 */
bool nmea_parse_gpgll_and_format(const char* gpgll_sentence, char* out_buf, size_t out_buf_size) {
    // Validate input parameters: check for NULL pointers or zero output buffer size.
    if (gpgll_sentence == NULL || out_buf == NULL || out_buf_size == 0) {
        return false; // Invalid parameters.
    }
    out_buf[0] = '\0'; // Initialize output buffer to an empty string to ensure defined state on early exit.

    // Verify that the input sentence starts with the GPGLL prefix.
    if (strncmp(gpgll_sentence, NMEA_GPGLL_PREFIX_INTERNAL, NMEA_GPGLL_PREFIX_LEN_INTERNAL) != 0) {
        return false; // Not a GPGLL sentence.
    }

    // Create a temporary modifiable copy of the input sentence for tokenization.
    // The size is NMEA_PARSER_MAX_GPGLL_CONTENT_LEN (defined in .h, for content after prefix)
    // plus prefix length, plus one for null terminator.
    char temp_sentence[NMEA_PARSER_MAX_GPGLL_CONTENT_LEN + NMEA_GPGLL_PREFIX_LEN_INTERNAL + 1];
    strncpy(temp_sentence, gpgll_sentence, sizeof(temp_sentence) - 1);
    temp_sentence[sizeof(temp_sentence) - 1] = '\0'; // Ensure null termination.

    // Array to store pointers to the start of each tokenized field.
    // Initialize all field pointers to point to an empty string as a default.
    char* fields[GPGLL_MAX_FIELDS];
    for (int i = 0; i < GPGLL_MAX_FIELDS; ++i) fields[i] = ""; 

    // Manual tokenization of the GPGLL sentence content (after the "$GPGLL," prefix).
    // `tokenizer_ptr` iterates through the sentence.
    // `field_start` points to the beginning of the current field.
    char *tokenizer_ptr = temp_sentence + NMEA_GPGLL_PREFIX_LEN_INTERNAL; // Start tokenizing after the prefix.
    int field_count = 0;        // Counter for the number of fields found.
    char *field_start = tokenizer_ptr; // Current field starts here.

    // Loop through the sentence character by character.
    while (*tokenizer_ptr && field_count < GPGLL_MAX_FIELDS) {
        // Check for delimiters: comma (',') or asterisk ('*').
        if (*tokenizer_ptr == ',' || *tokenizer_ptr == '*') { 
            char delimiter = *tokenizer_ptr; // Store the delimiter type.
            *tokenizer_ptr = '\0';           // Replace delimiter with null terminator to end current field string.
            fields[field_count++] = field_start; // Store pointer to the start of this field.
            field_start = tokenizer_ptr + 1;   // Next field starts after the delimiter.
            if (delimiter == '*') break;       // Asterisk marks the end of data fields (checksum follows).
        }
        tokenizer_ptr++; // Move to the next character.
    }
    // Handle the last field if it wasn't terminated by '*' and buffer space allows.
    // This captures the field between the last comma and the asterisk (or end of string if no asterisk).
    if (*field_start != '\0' && field_count < GPGLL_MAX_FIELDS && field_start < tokenizer_ptr) {
         fields[field_count++] = field_start;
    }

    // Extract pointers to specific fields based on their defined indices.
    // If a field was not present, its pointer in `fields` still points to the default empty string.
    const char* lat_val_str = fields[GPGLL_FIELD_LAT_VAL];
    const char* lat_dir_str = fields[GPGLL_FIELD_LAT_DIR];
    const char* lon_val_str = fields[GPGLL_FIELD_LON_VAL];
    const char* lon_dir_str = fields[GPGLL_FIELD_LON_DIR];
    // For UTC time, check if enough fields were parsed to include it.
    const char* utc_time_str = (field_count > GPGLL_FIELD_UTC_TIME) ? fields[GPGLL_FIELD_UTC_TIME] : "";

    // Format the UTC time to local time.
    char time_output_str[NMEA_PARSER_MAX_TIME_STR_LEN]; // Buffer for "HH:MM:SS"
    format_local_time(utc_time_str, time_output_str, sizeof(time_output_str));

    // --- Prepare Latitude Output String ---
    char lat_output_str[NMEA_PARSER_MAX_COORD_STR_LEN]; // Buffer for "Lat: DD.dddddd deg, N/S"
    // Check if latitude value string has content.
    bool lat_val_is_present = (strlen(lat_val_str) > 0);
    // Check if latitude direction string has content and is a valid character ('N' or 'S').
    bool lat_dir_is_valid_char = (strlen(lat_dir_str) > 0 && (lat_dir_str[0] == 'N' || lat_dir_str[0] == 'S'));

    if (lat_val_is_present) {
        // Convert NMEA latitude string to decimal degrees (2 digits for degrees part).
        double lat_degrees = convert_nmea_coord_to_degrees(lat_val_str, 2);
        // Determine direction character to display ('N', 'S', or '-' if invalid/missing).
        char lat_direction_to_display = (lat_dir_is_valid_char) ? lat_dir_str[0] : '-';
        // The original code had logic here to make lat_degrees negative for 'S'. This was removed.
        // The current version keeps lat_degrees positive and relies on the direction character.
        snprintf(lat_output_str, sizeof(lat_output_str), "Lat: %.6f deg, %c", lat_degrees, lat_direction_to_display);
    } else {
        // Latitude value is missing; use a placeholder string.
        snprintf(lat_output_str, sizeof(lat_output_str), "Lat: Waiting for data..., -");
    }

    // --- Prepare Longitude Output String ---
    char lon_output_str[NMEA_PARSER_MAX_COORD_STR_LEN]; // Buffer for "Long: DDD.dddddd deg, E/W"
    // Check if longitude value string has content.
    bool lon_val_is_present = (strlen(lon_val_str) > 0);
    // Check if longitude direction string has content and is a valid character ('E' or 'W').
    bool lon_dir_is_valid_char = (strlen(lon_dir_str) > 0 && (lon_dir_str[0] == 'E' || lon_dir_str[0] == 'W'));

    if (lon_val_is_present) {
        // Convert NMEA longitude string to decimal degrees (3 digits for degrees part).
        double lon_degrees = convert_nmea_coord_to_degrees(lon_val_str, 3);
        // Determine direction character to display ('E', 'W', or '-' if invalid/missing).
        char lon_direction_to_display = (lon_dir_is_valid_char) ? lon_dir_str[0] : '-';
        // The original code had logic here to make lon_degrees negative for 'W'. This was removed.
        // The current version keeps lon_degrees positive and relies on the direction character.
        snprintf(lon_output_str, sizeof(lon_output_str), "Long: %.6f deg, %c", lon_degrees, lon_direction_to_display);
    } else {
        // Longitude value is missing; use a placeholder string.
        snprintf(lon_output_str, sizeof(lon_output_str), "Long: Waiting for data..., -");
    }

    // --- Combine into the final output buffer ---
    // Format: "LocalTime | Latitude String | Longitude String\r\n"
    int written = snprintf(out_buf, out_buf_size, "%s | %s | %s%s",
                           time_output_str,    // Formatted local time
                           lat_output_str,     // Formatted latitude
                           lon_output_str,     // Formatted longitude
                           NMEA_LINE_ENDING_INTERNAL); // Append CRLF

    // Return true if snprintf was successful (written > 0) and the output was not truncated
    // (written < out_buf_size).
    return (written > 0 && (size_t)written < out_buf_size);
}