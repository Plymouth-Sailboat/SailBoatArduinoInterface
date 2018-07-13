#ifndef CONFIG_RC_H
#define CONFIG_RC_H

#include <Arduino.h>

/*MISC*/
/******/
//RC
#define RC_1 0
#define RC_2 1
#define RC_3 2
#define RC_4 3
#define RC_5 4
#define RC_6 5

#define RC_PIN_1 A8
#define RC_PIN_2 A9
#define RC_PIN_3 A10
#define RC_PIN_4 A11
#define RC_PIN_5 A12
#define RC_PIN_6 A13

//RC Config
#ifdef FLYSKY
#define RC_NUM_CHANNELS 4
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

#define RC_RUDDER	RC_1
#define RC_SAIL		RC_3
#define RC_AUTO		RC_2
#endif

#ifdef J5C01R
#define RC_NUM_CHANNELS 4
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

#define RC_RUDDER	RC_1
#define RC_SAIL		RC_3
#define RC_AUTO		RC_2
#endif

#ifdef HK_TR6
#define RC_NUM_CHANNELS 4
#define RC_1_MIN    1092
#define RC_1_MAX    1940
#define RC_2_MIN    1100
#define RC_2_MAX    1904
#define RC_3_MIN    1104
#define RC_3_MAX    1760
#define RC_4_MIN    1888
#define RC_4_MAX    1020
#define RC_5_MIN    0
#define RC_5_MAX    0
#define RC_6_MIN    0
#define RC_6_MAX    0

#define RC_RUDDER	RC_1
#define RC_SAIL		RC_3
#define RC_AUTO		RC_2
#endif

#endif
