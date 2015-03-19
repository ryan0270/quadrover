There are several directories in the project, but some of them were just for me to play around with ideas so aren't important.

## `$QUADROVER/arduino` ##
Contains the program to be run on the arduino. There are several different projects in here to test things, but the only one used in flight currently is `$QUADROVER/arduino/onboard/MotorInterface`

## `$QUADROVOER/Leash` ##
PC interface to talk to the phone over wifi. Right now this assumes the presence of a Vicon motion capture system since that has been my development platform. Even though I've achieved fully autonomous flight, i.e. without the Vicon information, I still haven't gotten around to updating this to make the Vicon options.

## `$QUADROVER/Rover` ##
The android application. This is explained in detail on the other wiki pages

## `$QUADROVER/ObsvTesting` ##
This is something I wrote to read a flight log file and simulate the exact same sensor events occurring at nearly the exact same time. This lets me test and tune my observers (position, velocity, attitude) on a PC using the real `Rover` classes and real data.