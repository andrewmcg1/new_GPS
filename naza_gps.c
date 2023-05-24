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

#define GPS_BUS 2

#define GPS_PAYLOAD_SIZE 58
#define COM_PAYLOAD_SIZE 58

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
        rc_read_bytes(GPS_BUS, buffer, 1);

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
    struct_to_int gps_union;
    static int16_t magXMax, magXMin, magYMax, magYMin;

    if (msgId == PAYLOAD_GPS)
    {
        uint8_t mask = 0;
        uint8_t mask_bits[8];
        uint8_t byte53_0, byte53_1, byte53_2, byte53_3;
        uint8_t byte61_4, byte61_5, byte61_6, byte61_7;

        byte53_0 = (gps_data.numberOfSatelites >> 7); 
        byte53_1 = (gps_data.numberOfSatelites >> 6) & 0x02; 
        byte53_2 = (gps_data.numberOfSatelites >> 5) & 0x04;
        byte53_3 = (gps_data.numberOfSatelites >> 4) & 0x08;

        byte61_4 = (gps_data.sequenceNumber[0] >> 3) & 0x10;
        byte61_5 = (gps_data.sequenceNumber[0] >> 2) & 0x20;
        byte61_6 = (gps_data.sequenceNumber[0] >> 1) & 0x40;
        byte61_7 = (gps_data.sequenceNumber[0]) & 0x40;
        
        mask_bits[0] = byte53_0 ^ byte61_4;
        mask_bits[1] = byte53_1 ^ byte61_5;
        mask_bits[2] = byte53_2 ^ byte61_6;
        mask_bits[3] = byte53_3 ^ byte61_7 ^ byte53_0;
        mask_bits[4] = byte53_1;
        mask_bits[4] = byte53_2;
        mask_bits[4] = byte53_3;
        mask_bits[4] = byte53_0 ^ byte61_4;

        for(int i = 0; i < 8; i++)
        {
            mask &= mask_bits[i];
        }

        gps_union.gps_structure = gps_data;
        
        for(int i = 0; i < GPS_PAYLOAD_SIZE; i++)
            gps_union.struct_as_int[i] ^= mask;

        gps_data = gps_union.structure;




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
