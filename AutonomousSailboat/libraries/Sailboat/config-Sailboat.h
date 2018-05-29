#define SAILBOAT
#define VERSION_ARDUINO "1.1"

/**CONTROLLERS**/
/***********/
#define NB_CONTROLLERS 8

#define STANDBY_CONTROLLER 0
#define RUDDERSAIL_CONTROLLER 1
#define RETURNHOME_CONTROLLER 2
#define HEADER_CONTROLLER 3
#define RC_CONTROLLER 4
#define SAILCAP_CONTROLLER 5
#define RUDDER_CONTROLLER 6
#define C_CONTROLLER 7

#ifdef SAILBOAT
#define FLYSKY
#include <config-SailboatSmall.h>
#endif
#ifdef SAILBOAT_BIG
#define J5C01R
#include <config-SailboatBig.h>
#endif

#include <config-RC.h>
