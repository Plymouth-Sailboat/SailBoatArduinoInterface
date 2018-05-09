#define SMALLBOAT

#ifdef SMALLBOAT
#include <config-SailboatSmall.h>
#elif SMALLBOAT_BIG
#include <config-SailboatBig.h>
#endif