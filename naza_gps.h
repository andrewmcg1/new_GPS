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

typedef struct __attribute__ ((packed)) gps_data_t
{
    byte dateAndTime[4];
    byte longitude[4];
    byte latitude[4];
    byte altitude[4];
    byte horrizontalAccuracyEstimate[4];
    byte verticalAccuracyEstimate[4];
    byte unknown1[4];
    byte NEDNorthVelocity[4];
    byte NEDEashVelocity[4];
    byte NEDDownVelocity[4];
    byte positionDOP[2];
    byte verticalDOP[2];
    byte northingDOP[2];
    byte eastingDOP[2];
    byte numberOfSatalites;
    byte unknown2;
    byte fixType;
    byte unknown3;
    byte fixStatusFlag;
    byte unknown4[2];
    byte xorMask;
    byte sequenceNumber[2];
} gps_data_t;

extern gps_data_t gps_data;

int gps_init();
int gps_getData();