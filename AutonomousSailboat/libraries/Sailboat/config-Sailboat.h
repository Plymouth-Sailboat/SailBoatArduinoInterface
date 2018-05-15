#define SMALLBOAT

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

#ifdef SMALLBOAT
#include <config-SailboatSmall.h>
#endif
#ifdef SMALLBOAT_BIG
#include <config-SailboatBig.h>
#endif