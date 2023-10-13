# Introduction

This repository contains FRC team 2930's 2023 Charged Up Competition code

Our Robot, named *Rober*, includes features such as:

- 4 times Autonomous Award Winner
------------------------------------
- 3 Game piece autonomous modes
- Fully automated aligment to human player feeder station
- Fully automated aligment to grid
-------------------------------------
- Full field vision using PhotonVision, 3 cameras, 2 Orange pi 5
- Multi-tag PNP support
------------------------------------------
- Simulated vision  (note: does not support multi tag)
- Simluated swerve (per 3061-lib)
- Simluated Vertical and Horizontal Elevators
------------------------------------------
- Logging and replay via [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/README.md)

------------------------------
-------------------------------

## Noteable Files
- [`.Robot.java`](/src/main/java/frc/robot/Robot.java)

- [`SwerveAutos.java`](/src/main/java/frc/robot/autonomous/SwerveAutos.java) - Competation autos

- [`DriverAssistAutos.java`](/src/main/java/frc/robot/DriverAssistAutos.java) - Teleop driver assist features

- [`SimulatedMechanism`](src/main/java/frc/robot/subsystems/SimMechanism/SimulatedMechanism.java) - Simulated Elevators

- [`/vision/`](/src/main/java/frc/lib/team3061/vision) - Vision system

------------------------------------
------------------------------------

## Credits & Refernces
- 3061-lib for the swerve library and base advantage kit structuring
- Team 6328 Mechanical Advantage build [thread](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2023-build-thread/420691)

**3061-lib credits**
* MK4/MK4i code initially from Team 364's [BaseFalconSwerve](https://github.com/Team364/BaseFalconSwerve)
* general AdvantageKit logging code, AdvantageKit-enabled Gyro classes, swerve module simulation, and drive characterization from Mechanical Advantage's [SwerveDevelopment](https://github.com/Mechanical-Advantage/SwerveDevelopment)
* AdvantageKit-enabled pneumatics classes from Mechanical Advantage's 2022 [robot code](https://github.com/Mechanical-Advantage/RobotCode2022)
* Talon factories from Citrus Circuits 2022 [robot code](https://github.com/frc1678/C2022)
* CAN device finder code from team 3620 2020 [robot code](https://github.com/FRC3620/FRC3620_2020_GalacticSenate)
* Setting up Spotless code linting [WPILib Spotless setup](https://docs.wpilib.org/en/latest/docs/software/advanced-gradlerio/code-formatting.html#spotless)
