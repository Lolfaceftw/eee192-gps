#include "nmea_parser.h" // Includes NMEA_PARSER_USE_FAKE_GPGLL_DATA and constants
#include <string.h>
#include <stdio.h>
#include <stdlib.h> // For atoi, strtod

// NMEA sentence specifics used internally by the parser
#define NMEA_GPGLL_PREFIX_INTERNAL "$GPGLL,"
#define NMEA_GPGLL_PREFIX_LEN_INTERNAL (sizeof(NMEA_GPGLL_PREFIX_INTERNAL) - 1)
#define NMEA_PARSER_OUTPUT_LINE_ENDING "\r\n" // Standard line ending for the output string

// GPGLL Field Indices (relative to start of data after "$GPGLL,")
#define GPGLL_FIELD_LAT_VAL    0
#define GPGLL_FIELD_LAT_DIR    1
#define GPGLL_FIELD_LON_VAL    2
#define GPGLL_FIELD_LON_DIR    3
#define GPGLL_FIELD_UTC_TIME   4
#define GPGLL_MAX_FIELDS       7

#define LOCAL_TIMEZONE_OFFSET_HOURS 8

/**
 * @brief Helper function to parse UTC time string and format to local time.
 */
static bool format_local_time(const char* utc_time_str, char* out_buf, size_t out_buf_size) {
    if (!utc_time_str || strlen(utc_time_str) < 6) {
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
    int hour_local = hour_utc + LOCAL_TIMEZONE_OFFSET_HOURS;

    if (hour_local >= 24) {
        hour_local -= 24;
    } else if (hour_local < 0) {
        hour_local += 24;
    }

    snprintf(out_buf, out_buf_size, "%02d:%02d:%02d", hour_local, minute, second);
    return true;
}

/**
 * @brief Helper function to convert NMEA coordinate string (DDmm.mm... or DDDmm.mm...) to decimal degrees.
 *
 * Example Latitude format: "1405.1234" (DD=14, mm.mm=05.1234) -> 14 + (05.1234 / 60)
 * Example Longitude format: "12102.5678" (DDD=121, mm.mm=02.5678) -> 121 + (02.5678 / 60)
 *
 * @param nmea_val_str The NMEA coordinate string (expected to be null-terminated).
 * @param out_decimal_degrees Pointer to a double where the converted decimal degrees will be stored.
 * @return true if conversion was successful, false otherwise.
 */
static bool nmea_ddm_to_decimal(const char* nmea_val_str, double* out_decimal_degrees) {
    if (!nmea_val_str || *nmea_val_str == '\0') {
        return false;
    }

    const char* dot_ptr = strchr(nmea_val_str, '.');
    int num_integer_digits_before_dot;

    if (dot_ptr) {
        num_integer_digits_before_dot = dot_ptr - nmea_val_str;
    } else {
        num_integer_digits_before_dot = strlen(nmea_val_str); // No decimal point
    }

    // There must be at least 2 digits for the "mm" part of minutes.
    // e.g., "DDmm" or "DDmm.m"
    if (num_integer_digits_before_dot < 2) {
        return false; // Not enough digits for "mm" (e.g., "1.234" or "D.mmm" or "1")
    }

    int deg_chars_len = num_integer_digits_before_dot - 2; // Number of characters for the degrees part (DD or DDD)

    double degrees_component = 0.0;
    const char* minutes_str_start;

    if (deg_chars_len < 0) { // Should be caught by num_integer_digits_before_dot < 2, but as a safeguard.
        return false;
    }
    
    if (deg_chars_len > 0) {
        char deg_str_buf[8]; // Sufficient for "DDD..."
        if (deg_chars_len >= (int)sizeof(deg_str_buf)) { // Cast to int to avoid warning with size_t
            return false; // Degrees part unexpectedly long
        }
        strncpy(deg_str_buf, nmea_val_str, deg_chars_len);
        deg_str_buf[deg_chars_len] = '\0';
        
        char* deg_end_ptr;
        degrees_component = strtod(deg_str_buf, deg_end_ptr);
        if (deg_end_ptr == deg_str_buf || *deg_end_ptr != '\0') { 
            return false; // Conversion failed or junk after degrees part
        }
    }
    // If deg_chars_len is 0, degrees_component remains 0.0 (e.g. "mm.mmmm" format)

    minutes_str_start = nmea_val_str + deg_chars_len;
    if (*minutes_str_start == '\0') { // Should not happen if num_integer_digits_before_dot >= 2
        return false; 
    }
    
    char* min_end_ptr;
    double minutes_component = strtod(minutes_str_start, &min_end_ptr);

    if (min_end_ptr == minutes_str_start || *min_end_ptr != '\0') {
        // Conversion failed or junk after minutes part (e.g. "02.56A")
        // Note: NMEA fields might have checksums like *XX, but the tokenizer should give us clean fields.
        return false; 
    }

    if (minutes_component < 0.0 || minutes_component >= 60.0) {
        // Allow minutes_component == 60.0 if it's exactly 60.0 and there's no fractional part?
        // NMEA standard implies minutes are 00.000 to 59.999. So >= 60.0 is an error.
        return false; // Invalid minutes range
    }

    *out_decimal_degrees = degrees_component + (minutes_component / 60.0);
    return true;
}


bool nmea_parse_gpgll_and_format(const char* gpgll_sentence, char* out_buf, size_t out_buf_size) {
    if (out_buf == NULL || out_buf_size == 0) {
        return false;
    }
    out_buf[0] = '\0';

    const char* lat_val_str_src;
    const char* lat_dir_str_src;
    const char* lon_val_str_src;
    const char* lon_dir_str_src;
    const char* utc_time_str_src;

    // Use const char* for string literals
    const char* fake_lat_val = "1405.1234"; 
    const char* fake_lat_dir = "N";
    const char* fake_lon_val = "12102.5678";
    const char* fake_lon_dir = "E";
    const char* fake_utc_time = "013045.00";

    if (NMEA_PARSER_USE_FAKE_GPGLL_DATA) {
        lat_val_str_src = fake_lat_val;
        lat_dir_str_src = fake_lat_dir;
        lon_val_str_src = fake_lon_val;
        lon_dir_str_src = fake_lon_dir;
        utc_time_str_src = fake_utc_time;
    } else {
        if (gpgll_sentence == NULL ||
            strncmp(gpgll_sentence, NMEA_GPGLL_PREFIX_INTERNAL, NMEA_GPGLL_PREFIX_LEN_INTERNAL) != 0) {
            return false;
        }

        char temp_sentence[NMEA_PARSER_MAX_GPGLL_CONTENT_LEN + NMEA_GPGLL_PREFIX_LEN_INTERNAL + 1]; 
        strncpy(temp_sentence, gpgll_sentence, sizeof(temp_sentence) - 1);
        temp_sentence[sizeof(temp_sentence) - 1] = '\0';

        char* fields[GPGLL_MAX_FIELDS];
        for (int i = 0; i < GPGLL_MAX_FIELDS; ++i) fields[i] = ""; // Initialize to empty strings

        char *tokenizer_ptr = temp_sentence + NMEA_GPGLL_PREFIX_LEN_INTERNAL;
        int field_count = 0;
        char *field_start = tokenizer_ptr;

        while (*tokenizer_ptr && field_count < GPGLL_MAX_FIELDS) {
            if (*tokenizer_ptr == ',' || *tokenizer_ptr == '*') { // GPGLL fields can end with data status or checksum
                char delimiter = *tokenizer_ptr;
                *tokenizer_ptr = '\0';
                fields[field_count++] = field_start;
                field_start = tokenizer_ptr + 1;
                if (delimiter == '*') break; // Stop tokenizing at checksum
            }
            tokenizer_ptr++;
        }
        // Capture the last field if it wasn't followed by a delimiter processed in the loop
        // (e.g. if the sentence ends right after the last relevant field before checksum)
        if (*field_start != '\0' && field_count < GPGLL_MAX_FIELDS && field_start < tokenizer_ptr) {
             fields[field_count++] = field_start;
        }

        lat_val_str_src = fields[GPGLL_FIELD_LAT_VAL];
        lat_dir_str_src = fields[GPGLL_FIELD_LAT_DIR];
        lon_val_str_src = fields[GPGLL_FIELD_LON_VAL];
        lon_dir_str_src = fields[GPGLL_FIELD_LON_DIR];
        utc_time_str_src = (field_count > GPGLL_FIELD_UTC_TIME) ? fields[GPGLL_FIELD_UTC_TIME] : "";
    }

    char time_output_str[NMEA_PARSER_MAX_TIME_STR_LEN];
    format_local_time(utc_time_str_src, time_output_str, sizeof(time_output_str));

    // --- Format Latitude ---
    char lat_output_str[NMEA_PARSER_MAX_COORD_STR_LEN];
    bool lat_has_value = (lat_val_str_src && strlen(lat_val_str_src) > 0);
    bool lat_has_direction = (lat_dir_str_src && strlen(lat_dir_str_src) > 0 && (lat_dir_str_src[0] == 'N' || lat_dir_str_src[0] == 'S'));
    double lat_decimal_degrees;

    // Build the format string for snprintf dynamically for precision
    char lat_format_str[32]; // Buffer for "Lat: %.<precision>f %c" or similar

    if (lat_has_value) {
        if (nmea_ddm_to_decimal(lat_val_str_src, &lat_decimal_degrees)) { // Conversion successful
            if (lat_has_direction) {
                snprintf(lat_format_str, sizeof(lat_format_str), "Lat: %%.%df %%c", NMEA_PARSER_DECIMAL_DEG_PRECISION);
                snprintf(lat_output_str, sizeof(lat_output_str), lat_format_str, lat_decimal_degrees, lat_dir_str_src[0]);
            } else {
                snprintf(lat_format_str, sizeof(lat_format_str), "Lat: %%.%df", NMEA_PARSER_DECIMAL_DEG_PRECISION);
                snprintf(lat_output_str, sizeof(lat_output_str), lat_format_str, lat_decimal_degrees);
            }
        } else { // Conversion failed, use original string format
            if (lat_has_direction) {
                snprintf(lat_output_str, sizeof(lat_output_str), "Lat: %s, %c", lat_val_str_src, lat_dir_str_src[0]);
            } else {
                snprintf(lat_output_str, sizeof(lat_output_str), "Lat: %s", lat_val_str_src);
            }
        }
    } else {
        snprintf(lat_output_str, sizeof(lat_output_str), "Lat: Waiting...");
    }

    // --- Format Longitude ---
    char lon_output_str[NMEA_PARSER_MAX_COORD_STR_LEN];
    bool lon_has_value = (lon_val_str_src && strlen(lon_val_str_src) > 0);
    bool lon_has_direction = (lon_dir_str_src && strlen(lon_dir_str_src) > 0 && (lon_dir_str_src[0] == 'E' || lon_dir_str_src[0] == 'W'));
    double lon_decimal_degrees;

    char lon_format_str[32]; // Buffer for "Long: %.<precision>f %c" or similar

    if (lon_has_value) {
        if (nmea_ddm_to_decimal(lon_val_str_src, &lon_decimal_degrees)) { // Conversion successful
            if (lon_has_direction) {
                snprintf(lon_format_str, sizeof(lon_format_str), "Long: %%.%df %%c", NMEA_PARSER_DECIMAL_DEG_PRECISION);
                snprintf(lon_output_str, sizeof(lon_output_str), lon_format_str, lon_decimal_degrees, lon_dir_str_src[0]);
            } else {
                snprintf(lon_format_str, sizeof(lon_format_str), "Long: %%.%df", NMEA_PARSER_DECIMAL_DEG_PRECISION);
                snprintf(lon_output_str, sizeof(lon_output_str), lon_format_str, lon_decimal_degrees);
            }
        } else { // Conversion failed, use original string format
            if (lon_has_direction) {
                snprintf(lon_output_str, sizeof(lon_output_str), "Long: %s, %c", lon_val_str_src, lon_dir_str_src[0]);
            } else {
                snprintf(lon_output_str, sizeof(lon_output_str), "Long: %s", lon_val_str_src);
            }
        }
    } else {
        snprintf(lon_output_str, sizeof(lon_output_str), "Long: Waiting...");
    }

    // Switched order: Time | Latitude | Longitude
    int written = snprintf(out_buf, out_buf_size, "%s | %s | %s%s",
                           time_output_str,
                           lat_output_str, // Latitude now second
                           lon_output_str, // Longitude now third
                           NMEA_PARSER_OUTPUT_LINE_ENDING);

    return (written > 0 && (size_t)written < out_buf_size);
}