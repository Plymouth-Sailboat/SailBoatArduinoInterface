/**
 * @file
 * @brief   Boat controller using a line-following method Header file
 *
 * Sail-boat controller for autonomous sail-boat - annotated version
 * Algorithm: https://www.ensta-bretagne.fr/jaulin/paper_jaulin_irsc12.pdf
 *
 * This controller is realized with approximations like the fact that locally the earth is flat
 * This approximation works as long as the boat's journey does not exceed 100km, if it does, 
 * you need to change the way the GPS coordinates are flatten (see in GPS.h/.cpp files)
 */


#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H
			  
class ControllerInterface{
  public:
/**
 * Initialization of the line-following algorithm:
 *  loading the Path and changing the reference to a local one
 *
 * @param initialPath: Initial table containing the list of way-points ([NB_POS_GPS]x[2] table)
 * @param newPath: Final table containing the list corrected of way-points ([NB_POS_GPS + 1]x[2] table)
 */
virtual void init() = 0;
  
/**
 * Controller for the line-following of the sail-boat:
 *
 * @param posActual: Actual GPS position in the local reference ([2] table)
 * @param heading: Angle between the boat and the North
 * @param windAngle: Angle from which the wind comes (with reference to the North)
 * @param posA: Former position of the aim
 * @param posB: New aim
 * @param rudder: Pointer to give the rudder's command
 * @param sail: Pointer to give the sail's command
 */
virtual void Control() = 0;
  private:
};

#endif
