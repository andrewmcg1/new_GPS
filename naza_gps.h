/**
 * <naza_gps.h>
 *
 * @brief Data structurs and functions to obtain gps readings from DJI Naza V2 Gps
 *
 * Development for this module is based on <a
 * href="https://www.rcgroups.com/forums/showthread.php?1995704-DJI-NAZA-GPS-communication-protocol-NazaDecoder-Arduino-library"
 * target="_blank">this forum post.</a>
 *
 * @author Glen Haggin (ghaggin@umich.edu)
 *
 * @addtogroup NazaGps
 * @{
 */

typedef uint8_t byte;

#ifndef __NAZA_GPS__
#define __NAZA_GPS__

#include <stdint.h>

#include <coordinates.h>

typedef union struct_to_int
{
    gps_data gps_structure;
    gps_compass_data compass_structure;
    uint8_t struct_as_int[GPS_PAYLOAD_SIZE];
} struct_to_int;

enum _ParseState
{
    START_BYTE_1,
    START_BYTE_2,
    MESSAGE_ID,
    VALIDATE_PAYLOAD,
    RECIEVE_PAYLOAD,
    CHECKSUM_1,
    CHECKSUM_2,
    DECODE_PAYLOAD
};
typedef _ParseState ParseState

typedef struct gps_data_t
{
    long int dateAndTime;
    long int longitude;
    long int latitude;
    long int altitude;
    long int horrizontalAccuracyEstimate;
    long int verticalAccuracyEstimate;
    long int unknown1;
    long int NEDNorthVelocity;
    long int NEDEashVelocity;
    long int NEDDownVelocity;
    short int positionDOP;
    short int verticalDOP;
    short int northingDOP;
    short int eastingDOP;
    byte numberOfSatalites;
    byte unknown2;
    byte fixType;
    byte unknown3;
    byte fixStatusFlag;
    short int unknown4;
    byte xorMask;
    short int sequenceNumber;
} gps_data_raw_t;

typedef struct compass_data_raw_t
{
    short int x_vector;
    short int y_vector;
    short int z_vector;
} compass_data_raw_t;

typedef struct gps_data_t
{
    lla_t lla;
    ned_waypoint_t ned;
    double spd;        ///< speed in m/s
    fixType_t fix;     ///< fix type
    uint8_t sat;       ///< number of satellites
    double headingNc;  ///< heading (not tilt compensated) in degrees
    double cog;        ///< course over ground
    double gpsVsi;     ///< vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
    double hdop;       ///< horizontal dilution of precision
    double vdop;       ///< vertical dilution of precision
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint64_t gps_data_received_ns;  ///< time on beaglebond board that gps is received
    uint8_t gps_valid;

} gps_data_t;

extern gps_data_t gps_data;

int gps_init();
int gps_getData();