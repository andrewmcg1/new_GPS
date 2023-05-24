/**
 * @file naza_gps.c
 **/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <rc/time.h>
#include <rc/uart.h>

#include <naza_gps.h>
#include <serial_com.h>
#include <state_estimator.h>

#define BAUDRATE    115200
#define TIMEOUT_S   .5
#define PAYLOAD_SIZE 64
#define GPS_BUS 2

#define GPS_STARTBYTE1 0x55       ///< first start byte
#define GPS_STARTBYTE2 0xAA       ///< second start byte
#define PAYLOAD_GPS 0x10          ///< gps message byte
#define PAYLOAD_GPS_LEN 0x3A      ///< gps message length
#define PAYLOAD_COMPASS 0x20      ///< compass message byte
#define PAYLOAD_COMPASS_LEN 0x06  ///< compass message length

gps_data_t gps_data;

/**
 * Functions only to be used locally
 */
static int __gps_parse(const int input);
static void __updateCS(const int input);
static void __gps_decode();

int gps_init()
{
    if (rc_uart_init(GPS_BUS, BAUDRATE, TIMEOUT_S, 0, 1, 0))
    {
        printf("Failed to open Serial Port\n");
        return -1;
    }

    // Set gps data to intialilly be invalid
    gps_data.gps_valid = 0;

    return 0;
}

int gps_getData()
{
    byte* buffer;
    if(rc_uart_bytes_available(GPS_BUS))
    {
        rc_read_bytes(GPS_BUS, buffer, PAYLOAD_SIZE);

        __gps_parse(gps_data)
    } 

    if (gps_data.gps_valid)
    {
        if (!origin.initialized)
        {
            set_origin(&gps_data.lla);
            origin.initialized = 1;
        }
        return 0;
    }

    return -1;
}

static int __gps_parse(const int input)
{
    static int count;
    uint8_t checkSum[2];
    static ParseState state = START_BYTE_1;
    static byte messageID;
    static byte messageLength;
    static unsigned char payload[PAYLOAD_SIZE];

    switch (state)
    {
    case START_BYTE_1:
        if (input == GPS_STARTBYTE1)
            state = START_BYTE_2;
        break;
        
    case START_BYTE_2:
        if (input == GPS_STARTBYTE2)
        {
            state = MESSAGE_ID;
            checkSum[0] = 0;
            checkSum[1] = 0;
        }
        else
            state = START_BYTE_1;
        break;
        
    case MESSAGE_ID:
        messageID = input;
        checkSum[0] += input;
        checkSum[1] += checkSum[0];
        state = VALIDATE_PAYLOAD;
        break;
        
    case VALIDATE_PAYLOAD:
        if ((messageID == PAYLOAD_GPS && input == PAYLOAD_GPS_LEN)
            || (messageID == PAYLOAD_COMPAS && input == PAYLOAD_COMPASS_LEN))
        {
            messageLength = input;
            count = 0;
            checkSum[0] += input;
            checkSum[1] += checkSum[0];
            state = RECIEVE_PAYLOAD;
        }
        else
            state = START_BYTE_1;
        break;
        
    case RECIEVE_PAYLOAD:
        payload[count++] = input;
        checkSum[0] += input;
        checkSum[1] += checkSum[0];
        if (count >= messageLength)
        {
            state = CHECKSUM_1;
        }
        break;

    case CHECKSUM_1:
        if (input == checkSum[0])
            state = CHECKSUM_2;
        else
            state = START_BYTE_1;
        break;

    case CHECKSUM_2:
        if (input == checkSum[1])
            state = DECODE_PAYLOAD;
        else
            state = START_BYTE_1;
        break;

    case DECODE_PAYLOAD:
        gps_data = *(gps_data_t*)payload;
        state = START_BYTE_1;
        __gps_decode();
    }

    return 0;
}

static void __gps_decode()
{
    static int16_t magXMax, magXMin, magYMax, magYMin;

    if (msgId == PAYLOAD_GPS)
    {
        uint8_t mask = payload[55];
        uint32_t time = __decodeLong(0, mask);
        gps_data.second = time & 0b00111111;
        time >>= 6;
        gps_data.minute = time & 0b00111111;
        time >>= 6;
        gps_data.hour = time & 0b00001111;
        time >>= 4;
        gps_data.day = time & 0b00011111;
        time >>= 5;
        if (gps_data.hour > 7) gps_data.day++;
        gps_data.month = time & 0b00001111;
        time >>= 4;
        gps_data.year = time & 0b01111111;

        gps_data.lla.lon = (double)__decodeLong(4, mask) / 10000000;
        gps_data.lla.lat = (double)__decodeLong(8, mask) / 10000000;
        gps_data.lla.alt = (double)__decodeLong(12, mask) / 1000;

        double nVel = (double)__decodeLong(28, mask) / 100;
        double eVel = (double)__decodeLong(32, mask) / 100;
        gps_data.spd = sqrt(nVel * nVel + eVel * eVel);
        gps_data.cog = atan2(eVel, nVel) * 180.0 / M_PI;
        if (gps_data.cog < 0) gps_data.cog += 360.0;
        gps_data.gpsVsi = -(double)__decodeLong(36, mask) / 100;
        gps_data.vdop = (double)__decodeShort(42, mask) / 100;
        double ndop = (double)__decodeShort(44, mask) / 100;
        double edop = (double)__decodeShort(46, mask) / 100;
        gps_data.hdop = sqrt(ndop * ndop + edop * edop);
        gps_data.sat = payload[48];
        uint8_t fixType = payload[50] ^ mask;
        uint8_t fixFlags = payload[52] ^ mask;
        switch (fixType)
        {
            case 2:
                gps_data.fix = FIX_2D;
                break;
            case 3:
                gps_data.fix = FIX_3D;
                break;
            default:
                gps_data.fix = NO_FIX;
                break;
        }
        if ((gps_data.fix != NO_FIX) && (fixFlags & 0x02))
        {
            gps_data.fix = FIX_DGPS;
        }

        // Convert current location to north east down coordinates;
        gps_data.ned = lla2ned(&gps_data.lla);

        // Log time that data is received
        gps_data.gps_data_received_ns = rc_nanos_since_epoch();

        // Check if gps is valid (> 4 satelites for 3D)
        gps_data.gps_valid = (gps_data.sat >= 4);
    }
    else if (msgId == PAYLOAD_COMPASS)
    {
        uint8_t mask = payload[4];
        mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^
               (((mask & 0x01) << 3) | ((mask & 0x01) << 7));
        int16_t x = __decodeShort(0, mask);
        int16_t y = __decodeShort(2, mask);
        if (x > magXMax) magXMax = x;
        if (x < magXMin) magXMin = x;
        if (y > magYMax) magYMax = y;
        if (y < magYMin) magYMin = y;
        gps_data.headingNc =
            -atan2(y - ((magYMax + magYMin) / 2), x - ((magXMax + magXMin) / 2)) * 180.0 / M_PI;
        if (gps_data.headingNc < 0)
        {
            gps_data.headingNc += 360.0;
        }
    }
    return;
}
