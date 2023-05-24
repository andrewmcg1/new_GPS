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

typedef struct gps_data_t
{
    byte dateAndTime[4];
    byte longitude[4];
    byte latitude[4];
    byte altitude[4];
} gps_data_t;

extern gps_data_t gps_data;