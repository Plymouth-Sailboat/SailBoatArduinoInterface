/*MISC*/
/******/
//RC
#define RC_1 0   // Pin 8 Connected to Channel-1 of Transmitter
#define RC_2 1   // Pin 9 Connected to Channel-2 of Transmitter
#define RC_3 2   // Pin 10 Connected to Channel-3 of Transmitter
#define RC_4 3   // Pin 11 Connected to Channel-4 of Transmitter
#define RC_5 4   // Pin 12 Connected to Channel-5 of Transmitter
#define RC_6 5   // Pin 13 Connected to Channel-6 of Transmitter

#define RC_PIN_1 8   // Pin 8 Connected to Channel-1 of Transmitter
#define RC_PIN_2 9   // Pin 9 Connected to Channel-2 of Transmitter
#define RC_PIN_3 10   // Pin 10 Connected to Channel-3 of Transmitter
#define RC_PIN_4 11   // Pin 11 Connected to Channel-4 of Transmitter
#define RC_PIN_5 12   // Pin 12 Connected to Channel-5 of Transmitter
#define RC_PIN_6 13   // Pin 13 Connected to Channel-6 of Transmitter
//RC Config

#ifdef FLYSKY
#define RC_1_MIN    1012
#define RC_1_MAX    1924
#define RC_2_MIN    1008
#define RC_2_MAX    1916
#define RC_3_MIN    1332
#define RC_3_MAX    1902
#define RC_4_MIN    912
#define RC_4_MAX    2050
#define RC_5_MIN    0
#define RC_5_MAX    0
#define RC_6_MIN    0
#define RC_6_MAX    0
#define RC_NUM_CHANNELS 6
#endif

#ifdef J5C01R
#define RC_1_MIN    1062
#define RC_1_MAX    2062
#define RC_2_MIN    1132
#define RC_2_MAX    2032
#define RC_3_MIN    0
#define RC_3_MAX    0
#define RC_4_MIN    0
#define RC_4_MAX    0
#define RC_5_MIN    0
#define RC_5_MAX    0
#define RC_6_MIN    0
#define RC_6_MAX    0
#define RC_NUM_CHANNELS 5
#endif
