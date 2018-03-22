[![Current version on Arduino](https://img.shields.io/badge/Arduino-v1.8.5-blue.svg)](https://www.arduino.cc/en/Main/Software)

# SailBoatArduinoInterface
Arduino code for [Plymouth's Autonomous Sailboat](http://165.227.238.42/). This contains the libraries, controllers and architecture of our arduino low-level controller for our sailboats. This arduino acquires all the data from the different sensors and send them to the main PC. It has also security features in case the PC shuts down.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

- Arduino IDE : https://www.arduino.cc/en/Main/Software
- Arduino Mega 2560 : (tested and working)
- Arduino Uno : 

### Installing

First copy the folder `libraries` and put it in `~/Documents/Arduino/` on your PC.

You should now be able to launch the .ino, compile and upload to the arduino.
This will launch the Arduino into Stanby Mode, just receiving data from the sensors and sending them to the PC. It won't and shouldn't move the rudder or the sail.

## Authors

* **Ulysse VAUTIER** - *Initial work* - [UlysseVautier](https://github.com/UlysseVautier)
* **Jian WAN** - [Jian Wan](https://www.plymouth.ac.uk/staff/jian-wan)
* **Christophe Viel** - [Christophe Viel](https://www.researchgate.net/profile/Christophe_Viel)

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

## Look at the Wiki!
If you want more information about the arduino and the boat, please look at [the wiki](https://github.com/Plymouth-Sailboat/SailBoatArduinoInterface/wiki)
