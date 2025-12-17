/**
 * @file nmea_parser.c
 * @brief NMEA 0183 sentence parser (FR-016)
 *
 * Parses standard NMEA sentences:
 * - GGA: Position, altitude, fix quality
 * - RMC: Position, speed, heading, date/time
 * - VTG: Speed and course
 * - GSA: DOP and active satellites
 */

#include "gps_navigation.h"
#include "fc_types.h"
#include "fc_log.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = TAG_GPS;

/* ==========================================================================
 * Internal Helpers
 * ========================================================================== */

/**
 * @brief Calculate NMEA checksum
 */
static uint8_t nmea_calculate_checksum(const char *sentence)
{
    uint8_t checksum = 0;
    const char *p = sentence;

    /* Skip leading $ */
    if (*p == '$') {
        p++;
    }

    /* XOR all characters until * or end */
    while (*p && *p != '*') {
        checksum ^= *p++;
    }

    return checksum;
}

/**
 * @brief Parse decimal degrees from NMEA format (DDDMM.MMMM)
 */
static double nmea_parse_coordinate(const char *str, char direction)
{
    if (str == NULL || *str == '\0') {
        return 0.0;
    }

    /* Find decimal point */
    const char *dot = strchr(str, '.');
    if (dot == NULL) {
        return 0.0;
    }

    /* Calculate number of degree digits */
    int deg_digits = (dot - str) - 2;
    if (deg_digits < 2 || deg_digits > 3) {
        return 0.0;
    }

    /* Parse degrees */
    char deg_str[4] = {0};
    strncpy(deg_str, str, deg_digits);
    double degrees = atof(deg_str);

    /* Parse minutes */
    double minutes = atof(str + deg_digits);

    /* Convert to decimal degrees */
    double result = degrees + minutes / 60.0;

    /* Apply direction */
    if (direction == 'S' || direction == 'W') {
        result = -result;
    }

    return result;
}

/**
 * @brief Get next field from NMEA sentence
 */
static const char *nmea_next_field(const char *sentence, char *field, size_t max_len)
{
    if (sentence == NULL || field == NULL) {
        return NULL;
    }

    size_t i = 0;
    while (*sentence && *sentence != ',' && *sentence != '*' && i < max_len - 1) {
        field[i++] = *sentence++;
    }
    field[i] = '\0';

    /* Skip separator */
    if (*sentence == ',' || *sentence == '*') {
        sentence++;
    }

    return sentence;
}

/* ==========================================================================
 * Public API
 * ========================================================================== */

bool nmea_validate_checksum(const char *sentence)
{
    if (sentence == NULL || sentence[0] != '$') {
        return false;
    }

    /* Find checksum marker */
    const char *star = strchr(sentence, '*');
    if (star == NULL || strlen(star) < 3) {
        return false;
    }

    /* Parse provided checksum (hex) */
    char hex[3] = {star[1], star[2], '\0'};
    uint8_t provided = (uint8_t)strtol(hex, NULL, 16);

    /* Calculate and compare */
    uint8_t calculated = nmea_calculate_checksum(sentence);

    return calculated == provided;
}

nmea_type_t nmea_parse_sentence(const char *sentence, gps_position_t *position)
{
    if (sentence == NULL || position == NULL) {
        return NMEA_UNKNOWN;
    }

    /* Validate checksum first */
    if (!nmea_validate_checksum(sentence)) {
        ESP_LOGD(TAG, "NMEA checksum failed");
        return NMEA_UNKNOWN;
    }

    /* Identify sentence type */
    if (strncmp(sentence, "$GPGGA", 6) == 0 ||
        strncmp(sentence, "$GNGGA", 6) == 0) {
        return nmea_parse_gga(sentence, position);
    }
    else if (strncmp(sentence, "$GPRMC", 6) == 0 ||
             strncmp(sentence, "$GNRMC", 6) == 0) {
        return nmea_parse_rmc(sentence, position);
    }
    else if (strncmp(sentence, "$GPVTG", 6) == 0 ||
             strncmp(sentence, "$GNVTG", 6) == 0) {
        return nmea_parse_vtg(sentence, position);
    }
    else if (strncmp(sentence, "$GPGSA", 6) == 0 ||
             strncmp(sentence, "$GNGSA", 6) == 0) {
        return nmea_parse_gsa(sentence, position);
    }

    return NMEA_UNKNOWN;
}

/**
 * @brief Parse GGA sentence (position fix)
 *
 * Format: $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 */
static nmea_type_t nmea_parse_gga(const char *sentence, gps_position_t *position)
{
    const char *p = sentence + 7; /* Skip "$GPGGA," */
    char field[20];

    /* Field 1: Time (hhmmss.ss) */
    p = nmea_next_field(p, field, sizeof(field));

    /* Field 2: Latitude */
    p = nmea_next_field(p, field, sizeof(field));
    char lat_str[20];
    strncpy(lat_str, field, sizeof(lat_str));

    /* Field 3: N/S */
    p = nmea_next_field(p, field, sizeof(field));
    char lat_dir = field[0];

    /* Field 4: Longitude */
    p = nmea_next_field(p, field, sizeof(field));
    char lon_str[20];
    strncpy(lon_str, field, sizeof(lon_str));

    /* Field 5: E/W */
    p = nmea_next_field(p, field, sizeof(field));
    char lon_dir = field[0];

    /* Field 6: Fix quality (0=invalid, 1=GPS, 2=DGPS) */
    p = nmea_next_field(p, field, sizeof(field));
    int fix_quality = atoi(field);

    /* Field 7: Number of satellites */
    p = nmea_next_field(p, field, sizeof(field));
    int satellites = atoi(field);

    /* Field 8: HDOP */
    p = nmea_next_field(p, field, sizeof(field));
    float hdop = atof(field);

    /* Field 9: Altitude */
    p = nmea_next_field(p, field, sizeof(field));
    float altitude = atof(field);

    /* Update position if valid fix */
    if (fix_quality > 0) {
        position->latitude_deg = nmea_parse_coordinate(lat_str, lat_dir);
        position->longitude_deg = nmea_parse_coordinate(lon_str, lon_dir);
        position->altitude_m = altitude;
        position->satellites = (uint8_t)satellites;
        position->hdop = hdop;
        position->valid = (satellites >= GPS_MIN_SATELLITES && hdop < GPS_HDOP_MAX);
    }

    return NMEA_GGA;
}

/**
 * @brief Parse RMC sentence (recommended minimum)
 *
 * Format: $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
 */
static nmea_type_t nmea_parse_rmc(const char *sentence, gps_position_t *position)
{
    const char *p = sentence + 7; /* Skip "$GPRMC," */
    char field[20];

    /* Field 1: Time */
    p = nmea_next_field(p, field, sizeof(field));

    /* Field 2: Status (A=valid, V=void) */
    p = nmea_next_field(p, field, sizeof(field));
    bool status_valid = (field[0] == 'A');

    /* Field 3: Latitude */
    p = nmea_next_field(p, field, sizeof(field));
    char lat_str[20];
    strncpy(lat_str, field, sizeof(lat_str));

    /* Field 4: N/S */
    p = nmea_next_field(p, field, sizeof(field));
    char lat_dir = field[0];

    /* Field 5: Longitude */
    p = nmea_next_field(p, field, sizeof(field));
    char lon_str[20];
    strncpy(lon_str, field, sizeof(lon_str));

    /* Field 6: E/W */
    p = nmea_next_field(p, field, sizeof(field));
    char lon_dir = field[0];

    /* Field 7: Speed over ground (knots) */
    p = nmea_next_field(p, field, sizeof(field));
    float speed_knots = atof(field);

    /* Field 8: Course over ground (degrees) */
    p = nmea_next_field(p, field, sizeof(field));
    float course_deg = atof(field);

    /* Update position if valid */
    if (status_valid) {
        position->latitude_deg = nmea_parse_coordinate(lat_str, lat_dir);
        position->longitude_deg = nmea_parse_coordinate(lon_str, lon_dir);
        position->speed_ms = speed_knots * 0.514444f; /* Knots to m/s */
        position->heading_deg = course_deg;
    }

    return NMEA_RMC;
}

/**
 * @brief Parse VTG sentence (velocity)
 */
static nmea_type_t nmea_parse_vtg(const char *sentence, gps_position_t *position)
{
    const char *p = sentence + 7;
    char field[20];

    /* Field 1: True course */
    p = nmea_next_field(p, field, sizeof(field));
    float true_course = atof(field);

    /* Skip fields 2-6 */
    for (int i = 0; i < 5; i++) {
        p = nmea_next_field(p, field, sizeof(field));
    }

    /* Field 7: Speed km/h */
    p = nmea_next_field(p, field, sizeof(field));
    float speed_kmh = atof(field);

    position->heading_deg = true_course;
    position->speed_ms = speed_kmh / 3.6f; /* km/h to m/s */

    return NMEA_VTG;
}

/**
 * @brief Parse GSA sentence (DOP and satellites)
 */
static nmea_type_t nmea_parse_gsa(const char *sentence, gps_position_t *position)
{
    const char *p = sentence + 7;
    char field[20];

    /* Field 1: Mode (A=auto, M=manual) */
    p = nmea_next_field(p, field, sizeof(field));

    /* Field 2: Fix type (1=no fix, 2=2D, 3=3D) */
    p = nmea_next_field(p, field, sizeof(field));
    int fix_type = atoi(field);

    /* Skip satellite PRN fields (3-14) */
    for (int i = 0; i < 12; i++) {
        p = nmea_next_field(p, field, sizeof(field));
    }

    /* Field 15: PDOP */
    p = nmea_next_field(p, field, sizeof(field));
    /* float pdop = atof(field); */

    /* Field 16: HDOP */
    p = nmea_next_field(p, field, sizeof(field));
    position->hdop = atof(field);

    /* Field 17: VDOP */
    p = nmea_next_field(p, field, sizeof(field));
    /* float vdop = atof(field); */

    position->valid = (fix_type >= 2);

    return NMEA_GSA;
}
