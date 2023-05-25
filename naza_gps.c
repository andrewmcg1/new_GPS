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

#define BAUDRATE    115200
#define TIMEOUT_S   .5

#define GPS_BUS 2

#define PAYLOAD_SIZE 64
#define GPS_PAYLOAD_LENGTH 58
#define COMPASS_PAYLOAD_LENGTH

#define GPS_STARTBYTE1 0x55       ///< first start byte
#define GPS_STARTBYTE2 0xAA       ///< second start byte
#define PAYLOAD_GPS 0x10          ///< gps message byte
#define PAYLOAD_COMPASS 0x20      ///< compass message byte

typedef struct RawGPSData
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
    unsigned char numberOfSatalites;
    unsigned char unknown2;
    unsigned char fixType;
    unsigned char unknown3;
    unsigned char fixStatusFlag;
    short int unknown4;
    unsigned char xorMask;
    short int sequenceNumber;
} RawGPSData;

typedef struct RawCompassData
{
    short int x_vector;
    short int y_vector;
    short int z_vector;
} RawCompassData;

typedef union struct_to_int
{
    RawGPSData gps_structure;
    RawCompassData compass_structure;
    uint8_t struct_as_int[GPS_PAYLOAD_LENGTH];
} struct_to_int;

gps_data_t gps_data;

RawGPSData gps_data_raw;
RawCompassData compass_data_raw;

/**
 * Functions only to be used locally
 */
static int __gps_parse(const int input);
static void __updateCS(const int input);
static void __gps_decode(unsigned char messageID);

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
    unsigned char* buffer;
    if(rc_uart_bytes_available(GPS_BUS))
    {
        rc_read_bytes(GPS_BUS, buffer, 1);

        __gps_parse(gps_data_raw);
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
    static ParseState parseState = START_BYTE_1;
    static unsigned char messageID;
    static unsigned char messageLength;
    static unsigned char payload[PAYLOAD_SIZE];

    switch (parseState)
    {
    case START_BYTE_1:
        if (input == GPS_STARTBYTE1)
            parseState = START_BYTE_2;
        break;
        
    case START_BYTE_2:
        if (input == GPS_STARTBYTE2)
        {
            parseState = MESSAGE_ID;
            checkSum[0] = 0;
            checkSum[1] = 0;
        }
        else
            parseState = START_BYTE_1;
        break;
        
    case MESSAGE_ID:
        messageID = input;
        checkSum[0] += input;
        checkSum[1] += checkSum[0];
        parseState = VALIDATE_PAYLOAD;
        break;
        
    case VALIDATE_PAYLOAD:
        if ((messageID == PAYLOAD_GPS && input == GPS_PAYLOAD_LENGTH)
            || (messageID == PAYLOAD_COMPASS && input == COMPASS_PAYLOAD_LENGTH))
        {
            messageLength = input;
            count = 0;
            checkSum[0] += input;
            checkSum[1] += checkSum[0];
            parseState = RECIEVE_PAYLOAD;
        }
        else
            parseState = START_BYTE_1;
        break;
        
    case RECIEVE_PAYLOAD:
        payload[count++] = input;
        checkSum[0] += input;
        checkSum[1] += checkSum[0];
        if (count >= messageLength)
        {
            parseState = CHECKSUM_1;
        }
        break;

    case CHECKSUM_1:
        if (input == checkSum[0])
            parseState = CHECKSUM_2;
        else
            parseState = START_BYTE_1;
        break;

    case CHECKSUM_2:
        if (input == checkSum[1])
            parseState = DECODE_PAYLOAD;
        else
            parseState = START_BYTE_1;
        break;

    case DECODE_PAYLOAD:
        if (messageID == PAYLOAD_GPS)
            gps_data_raw = *(RawGPSData*)payload;
        else if (messageID == PAYLOAD_COMPASS)
            compass_data_raw = *(RawCompassData*)payload;
        parseState = START_BYTE_1;
        __gps_decode(messageID);
    }

    return 0;
}

static void __gps_decode(unsigned char messageID)
{
    struct_to_int gps_union;
    static int16_t magXMax, magXMin, magYMax, magYMin;

    if (messageID == PAYLOAD_GPS)
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
        
        for(int i = 0; i < GPS_PAYLOAD_LENGTH; i++)
            gps_union.struct_as_int[i] ^= mask;

        gps_data = gps_union.structure;

        // Convert current location to north east down coordinates;
        gps_data.ned = lla2ned(&gps_data.lla);

        // Log time that data is received
        gps_data.gps_data_received_ns = rc_nanos_since_epoch();

        // Check if gps is valid (> 4 satelites for 3D)
        gps_data.gps_valid = (gps_data.sat >= 4);
    }
    else if (messageID == PAYLOAD_COMPASS)
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
