### Competition Robot Features

1. Autonomous
    1.  score from pre-set position at start of autonomous.


**Features**
----
* multiple robots, including a simulated robot with basic simulation of swerve modules
* logging and replay via [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/README.md)
* CAN FD (CANivore) to reduce CAN bus utilization
* reports devices missing from the CAN bus
* swerve-specific features
    * robot-relative and field-relative driving modes
    * current limiting configuration for motors
    * x-stance
    * leave wheels rotated in last direction when not driving to enable smooth continuation of motion
    * switch drive motors to coast mode when robot is disabled and has stopped moving to facilitate manual pushing


## remember to copy 3061 readme credits into the main credits section here
