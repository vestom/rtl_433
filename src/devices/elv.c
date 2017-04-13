#include "rtl_433.h"
#include "data.h"
#include "util.h"
#include <math.h>


uint16_t AD_POP(uint8_t bb[BITBUF_COLS], uint8_t bits, uint8_t bit) {
    uint16_t val = 0;
    uint8_t i, byte_no, bit_no;
    for (i=0;i<bits;i++) {
        byte_no=   (bit+i)/8 ;
        bit_no =7-((bit+i)%8);
        if (bb[byte_no]&(1<<bit_no)) val = val | (1<<i);
    }
    return val;
}

static int em1000_callback(bitbuffer_t *bitbuffer) {
    // based on fs20.c
    bitrow_t *bb = bitbuffer->bb;
    uint8_t dec[10];
    uint8_t bytes=0;
    uint8_t bit=18; // preamble
    uint8_t bb_p[14];
    char* types[] = {"S", "?", "GZ"};
    uint8_t checksum_calculated = 0;
    uint8_t i;
	uint8_t stopbit;
	uint8_t checksum_received;

    // check and combine the 3 repetitions
    for (i = 0; i < 14; i++) {
        if(bb[0][i]==bb[1][i] || bb[0][i]==bb[2][i]) bb_p[i]=bb[0][i];
        else if(bb[1][i]==bb[2][i])                  bb_p[i]=bb[1][i];
        else return 0;
    }

    // read 9 bytes with stopbit ...
    for (i = 0; i < 9; i++) {
        dec[i] = AD_POP (bb_p, 8, bit); bit+=8;
        stopbit=AD_POP (bb_p, 1, bit); bit+=1;
        if (!stopbit) {
//            fprintf(stdout, "!stopbit: %i\n", i);
            return 0;
        }
        checksum_calculated ^= dec[i];
        bytes++;
    }

    // Read checksum
    checksum_received = AD_POP (bb_p, 8, bit); bit+=8;
    if (checksum_received != checksum_calculated) {
//        fprintf(stdout, "checksum_received != checksum_calculated: %d %d\n", checksum_received, checksum_calculated);
        return 0;
    }

//for (i = 0; i < bytes; i++) fprintf(stdout, "%02X ", dec[i]); fprintf(stdout, "\n");

    // based on 15_CUL_EM.pm
    fprintf(stdout, "Energy sensor event:\n");
    fprintf(stdout, "protocol      = ELV EM 1000, %d bits\n",bitbuffer->bits_per_row[1]);
    fprintf(stdout, "type          = EM 1000-%s\n",dec[0]>=1&&dec[0]<=3?types[dec[0]-1]:"?");
    fprintf(stdout, "code          = %d\n",dec[1]);
    fprintf(stdout, "seqno         = %d\n",dec[2]);
    fprintf(stdout, "total cnt     = %d\n",dec[3]|dec[4]<<8);
    fprintf(stdout, "current cnt   = %d\n",dec[5]|dec[6]<<8);
    fprintf(stdout, "peak cnt      = %d\n",dec[7]|dec[8]<<8);

    return 1;
}
float get_ws2000_humidity(unsigned char *message, unsigned int sensor_id) {
    // sensor ID included to support sensors with humidity in different position
    float humidity = 0;
    humidity = message[7]*100 + message[6]*10 + message[5];
    humidity = humidity / 10;
    return humidity;
}
float get_ws2000_temperature(unsigned char *message, unsigned int sensor_id) {
    // sensor ID included  to support sensors with temp in different position
    float temp_c = 0;
    temp_c = (((message[4])*100)+ message[3]*10 + message[2]);
    temp_c = temp_c / 10;
    if (message[1] & 0x08)
        temp_c = -temp_c;
    return temp_c;
}
unsigned int get_ws2000_pressure(unsigned char *message, unsigned int sensor_id) {
    // sensor ID included to support sensors with humidity in different position
    unsigned int pressure = 0;
    pressure = 200 + message[10]*100 + message[9]*10 + message[8];
    return pressure;
}
unsigned int get_ws2000_rain_sum(unsigned char *message, unsigned int sensor_id) {
    // sensor ID included to support sensors with humidity in different position
    unsigned int rain_sum = 0;
    rain_sum = message[4]*16*16 + message[3]*16 + message[2];
    return rain_sum;
}
unsigned int get_ws2000_luminance(unsigned char *message, unsigned int sensor_id) {
    // sensor ID included to support sensors with humidity in different position
    unsigned int luminance = 0;
    luminance = (message[4]*100 + message[3]*10 + message[2])* powl(10, message[5]);
    return luminance;
}
unsigned int get_ws2000_sunminutes(unsigned char *message, unsigned int sensor_id) {
    // sensor ID included to support sensors with humidity in different position
    unsigned int sun_minutes = 0;
    sun_minutes = message[8]*16*16 + message[7]*16 + message[6];
    return sun_minutes;
}
float get_ws2000_windspeed(unsigned char *message, unsigned int sensor_id) {
    // sensor ID included  to support sensors with temp in different position
    float wind = 0;
    wind = (float) message[2]/10 + message[4]*10 + message[3];
    if (message[1] & 0x08)
        wind += 100;
    return wind;
}
unsigned int get_ws2000_direction(unsigned char *message, unsigned int sensor_id) {
    // sensor ID included to support sensors with humidity in different position
    unsigned int direction = 0;
    direction = (message[7]&3)*100 + message[6]*10 + message[5];
    return direction;
}
float get_ws2000_variation(unsigned char *message, unsigned int sensor_id) {
    // sensor ID included  to support sensors with temp in different position
    float variation = 0;
    variation = (float) (message[7]>>2) * 22.5;
    return variation;
}



static int ws2000_callback(bitbuffer_t *bitbuffer) {
    // based on http://www.dc3yc.privat.t-online.de/protocol.htm
    // and on (Anemometer & luminance) http://www.f6fbb.org/domo/sensors/
    bitrow_t *bb = bitbuffer->bb;
    uint8_t dec[13];
    uint8_t nibbles=0;
    uint8_t bit=11; // preamble
    char* types[]={"!AS3", "AS2000/ASH2000/S2000/S2001A/S2001IA/ASH2200/S300IA", "!S2000R", "!S2000W", "S2001I/S2001ID", "!S2500H", "!Pyrano", "!KS200/KS300"};
    uint8_t length[16]={5, 8, 5, 8, 12, 9, 8, 14};
    uint8_t check_calculated=0, sum_calculated=0;
    uint8_t i;
    uint8_t stopbit;
	uint8_t sum_received;
    data_t *data;
    char time_str[LOCAL_TIME_BUFLEN];
    local_time_str(0, time_str);

    dec[0] = AD_POP (bb[0], 4, bit); bit+=4;
    stopbit= AD_POP (bb[0], 1, bit); bit+=1;
    if (!stopbit) {
        if(debug_output) fprintf(stdout, "!stopbit\n");
        return 0;
    }
    check_calculated ^= dec[0];
    sum_calculated   += dec[0];

    // read nibbles with stopbit ...
    for (i = 1; i <= length[dec[0]]; i++) {
        dec[i] = AD_POP (bb[0], 4, bit); bit+=4;
        stopbit= AD_POP (bb[0], 1, bit); bit+=1;
        if (!stopbit) {
            if(debug_output) fprintf(stdout, "!stopbit %i\n", bit);
            return 0;
        }
        check_calculated ^= dec[i];
        sum_calculated   += dec[i];
        nibbles++;
    }
    if(debug_output) { for (i = 0; i < nibbles; i++) fprintf(stdout, "%02X ", dec[i]); fprintf(stdout, "\n"); }

    if (check_calculated) {
        if(debug_output) fprintf(stdout, "check_calculated (%d) != 0\n", check_calculated);
        return 0;
    }

    // Read sum
    sum_received = AD_POP (bb[0], 4, bit); bit+=4;
    sum_calculated+=5;
    sum_calculated&=0xF;
    if (sum_received != sum_calculated) {
        if(debug_output) fprintf(stdout, "sum_received (%d) != sum_calculated (%d) ", sum_received, sum_calculated);
        return 0;
    }
    if (dec[0]==0 || dec[0]==1 || dec[0]==4)
    {
        float humidity = 0;
        if (dec[0]>0)
            humidity = get_ws2000_humidity(dec, 1);
        if (humidity==0)
        {
            data = data_make(
                             "time",          "",            DATA_STRING, time_str,
                             "brand",         "Station",     DATA_STRING, "WS2000",
                             "model",         "Sensor",      DATA_STRING, "Temperature",
                             "address",       "Address",     DATA_INT,    (dec[1] & 7),
                             "temperature_C", "Celcius",     DATA_FORMAT, "%.1f C", DATA_DOUBLE, get_ws2000_temperature(dec, 1), // set 1 instead sensor_id
                             NULL);
            data_acquired_handler(data);
        }
        if (humidity>0.0 && dec[0] != 4)
        {
            data = data_make(
                             "time",          "",           DATA_STRING, time_str,
                             "brand",         "Station",    DATA_STRING, "WS2000",
                             "model",         "Sensor",     DATA_STRING, "Temperature/Humidity",
                             "address",       "Address",    DATA_INT,    (dec[1] & 7),
                             "temperature_C", "Celcius",    DATA_FORMAT, "%.1f C", DATA_DOUBLE, get_ws2000_temperature(dec, 1), // set 1 instead sensor_id
                             "humidity",      "Humidity",   DATA_FORMAT, "%.1f %%", DATA_DOUBLE, humidity,
                             NULL);
            data_acquired_handler(data);
        }
        if (humidity>0 && dec[0] == 4)
        {
            data = data_make(
                             "time",           "",           DATA_STRING, time_str,
                             "brand",          "Station",    DATA_STRING, "WS2000",
                             "model",          "Sensor",     DATA_STRING, "Temperature/Humidity/Pressure",
                             "address",        "Address",    DATA_INT,    (dec[1] & 7),
                             "temperature_C",  "Celcius",    DATA_FORMAT, "%.1f C", DATA_DOUBLE, get_ws2000_temperature(dec, 1), // set 1 instead sensor_id
                             "humidity",       "Humidity",   DATA_FORMAT, "%.1f %%", DATA_DOUBLE, humidity,
                             "pressure",       "Pressure",   DATA_FORMAT, "%u hPa", DATA_INT,get_ws2000_pressure(dec, 1),
                             NULL);
            data_acquired_handler(data);
        }

    }
    else if (dec[0]==2)   // rain
        {  data = data_make(
                         "time",           "",           DATA_STRING, time_str,
                         "brand",          "Station",    DATA_STRING, "WS2000",
                         "model",          "Sensor",     DATA_STRING, "Rain",
                         "address",        "Address",    DATA_INT,    (dec[1] & 7),
                         "rain_sum",       "Rain Counter",           DATA_INT,get_ws2000_rain_sum(dec, 1), // Counter for rain gauge impulses; reset to 0 after 4095
                         NULL);
            data_acquired_handler(data);
        

    }
    else if (dec[0]==3)   // wind
    {
        data = data_make(
                         "time",           "",           DATA_STRING, time_str,
                         "brand",          "Station",    DATA_STRING, "WS2000",
                         "model",          "Sensor",     DATA_STRING, "Wind",
                         "address",        "Address",    DATA_INT,    (dec[1] & 7),
                         "wind_speed",     "Wind",       DATA_FORMAT, "%.1f km/h", DATA_DOUBLE, get_ws2000_windspeed(dec, 1), // set 1 instead sensor_id
                         "direction",      "Direction",  DATA_FORMAT, "%u deg", DATA_INT, get_ws2000_direction(dec, 1),
                         "variation",      "Variation",  DATA_FORMAT, "%.01f deg", DATA_DOUBLE, get_ws2000_variation(dec, 1),
                         NULL);
        data_acquired_handler(data);


    } else if (dec[0]==5)   // luminance
    {
        data = data_make(
                         "time",           "",            DATA_STRING, time_str,
                         "brand",          "Station",     DATA_STRING, "WS2000",
                         "model",          "Sensor",      DATA_STRING, "Luminance",
                         "address",        "Address",     DATA_INT,    (dec[1] & 7),
                         "luminance",      "Luminance",   DATA_INT,get_ws2000_luminance(dec, 1),
                         "sun_minutes",    "Sun minutes", DATA_INT,get_ws2000_sunminutes(dec, 1), // Counter for minutes >20kLux reset to 0 after 4095
                         NULL);
        data_acquired_handler(data);
        
    } else if (dec[0]==6)   // pyrano (ToDo)
    {
    } else if (dec[0]==7)   // kombi (ToDo)
    {
    }

    //fprintf(stdout, "Weather station sensor event:\n");
    //fprintf(stdout, "protocol      = ELV WS 2000, %d bits\n",bitbuffer->bits_per_row[1]);
    //fprintf(stdout, "type (!=ToDo) = %s\n", dec[0]<=7?types[dec[0]]:"?");
    //fprintf(stdout, "code          = %d\n", dec[1]&7);
    //fprintf(stdout, "temp          = %s%d.%d\n", dec[1]&8?"-":"", dec[4]*10+dec[3], dec[2]);
    //fprintf(stdout, "humidity      = %d.%d\n", dec[7]*10+dec[6], dec[5]);
    //if(dec[0]==4) {
    //    fprintf(stdout, "pressure      = %d\n", 200+dec[10]*100+dec[9]*10+dec[8]);
    //}

    return 1;
}
/*
 * List of fields to output when using CSV
 *
 * Used to determine what fields will be output in what
 * order for this devince when using -F csv.
 *
 */
static char *csv_output_fields[] = {
    "time",
    "model",
    "id",
    "temperature_C",
    "humidity",
    "pressure",
    "rain_sum",
    "luminance",
    "wind_speed",
    "direction",
    NULL
};

r_device elv_em1000 = {
    .name           = "ELV EM 1000",
    .modulation     = OOK_PULSE_PPM_RAW,
    .short_limit    = 750,
    .long_limit     = 7250,
    .reset_limit    = 30000,
    .json_callback  = &em1000_callback,
    .disabled       = 1,
    .demod_arg      = 0,
};

r_device elv_ws2000 = {
    .name           = "ELV WS 2000",
    .modulation     = OOK_PULSE_PWM_RAW,
    .short_limit    = (854+366)/2,  // 0 => 854us, 1 => 366us according to link in top
    .long_limit     = 1000, // no repetitions
    .reset_limit    = 1000, // Longest pause is 854us according to link
    .json_callback  = &ws2000_callback,
    .fields         = csv_output_fields,
    .disabled       = 0,
    .demod_arg      = 0,
};
