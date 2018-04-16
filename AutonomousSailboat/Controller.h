/**
   @file
   @brief   Boat controller using a line-following method Header file

   Sail-boat controller for autonomous sail-boat - annotated version
   Algorithm: https://www.ensta-bretagne.fr/jaulin/paper_jaulin_irsc12.pdf

   This controller is realized with approximations like the fact that locally the earth is flat
   This approximation works as long as the boat's journey does not exceed 100km, if it does,
   you need to change the way the GPS coordinates are flatten (see in GPS.h/.cpp files)
*/


#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ControllerInterface.h>

class Controller : public ControllerInterface {
  public:
    Controller(int waypoints);
    /**
       Initialization of the line-following algorithm:
        loading the Path and changing the reference to a local one

       @param initialPath: Initial table containing the list of way-points ([NB_POS_GPS]x[2] table)
       @param newPath: Final table containing the list corrected of way-points ([NB_POS_GPS + 1]x[2] table)
    */
    void init();

    /**
       Controller for the line-following of the sail-boat:

       @param posActual: Actual GPS position in the local reference ([2] table)
       @param heading: Angle between the boat and the North
       @param windAngle: Angle from which the wind comes (with reference to the North)
       @param posA: Former position of the aim
       @param posB: New aim
       @param rudder: Pointer to give the rudder's command
       @param sail: Pointer to give the sail's command
    */
    void Control(const geometry_msgs::Twist& cmd);


  private:
    // Customization:
    // If the line-following is not accurate and does not converges to the line, uncomment the following line:
    // You must know that it will requires to tune 2 parameters: z and ALPHA defined later in file "Controller.cpp"
    //#define INTEGRATOR

    // Path following
    int NB_POS_GPS;  // Number of Way-points  // TODO: set this number automatically !

    int token;  // Number indicating the way-point we aim

    // Indicating the state of the global system:
    bool finished;

    // List of Way-points:
    double** initialPath;
    double** newPath;  // List of Way-points in the new reference
    
    /**
       Program which allows to follow a Path by changing the aim as we go along

       @param newPath: Table containing the list corrected of way-points ([NB_POS_GPS + 1]x[2] table)
       @param posActual: Actual GPS position in the local reference ([2] table)
       @param p_token: Token to indicate which way-point to aim
    */
    void pathFollowing();
};

#endif
