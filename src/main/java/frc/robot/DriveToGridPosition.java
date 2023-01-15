// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FollowPathOnTheFly;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.intake.Intake;

/** Add your docs here. */
public class DriveToGridPosition {
    Drivetrain drivetrain;
    Intake intake;

    CommandXboxController controller; 

    public static double drive_slow_test_vel = 0.2;
    public static double angular_slow_test_vel = 0.2;

    public DriveToGridPosition(Drivetrain drivetrain, Intake intake, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.controller = controller;
    }

    public Command driveToGridPosAndScore(){
        
        return new SequentialCommandGroup(
            new FollowPathOnTheFly(
                GridPositions.BOTTOM_ENTRANCE_CHECKPOINT.pose, 
                drivetrain, 
                drive_slow_test_vel, 
                angular_slow_test_vel),
            
            new FollowPathOnTheFly(GridPositions.GRID_8_LINEUP.pose, drivetrain, drive_slow_test_vel, angular_slow_test_vel),

            new FollowPathOnTheFly(GridPositions.GRID_8_SCORE.pose, drivetrain, drive_slow_test_vel, angular_slow_test_vel),
            
            Commands.run(() -> intake.extend(), intake), 

            Commands.waitSeconds(1),

            Commands.run(() -> intake.retract(), intake)
        );
        
    }

    public Command driveToCommunityCheckPointBasedOnPos(){
        /**
         * Bottom entrance constraints: 
         * x < 4.65 
         * 0.01 < y < 1.4
         * 
         * Top entrance constraints: 
         * x < 4.65 
         * 4.1 < y < 5.10 
         */

        var selectedEntrance = SelectedEntrance.UNKNOWN;

        var currentPos = drivetrain.getPose();
    
        //too far away
        if(currentPos.getX() > 4.65) {
            return errorRumbleControllerCommand();
        }

        //too far up or down OR in between the two entrances i.e in front of the pad 
        if(currentPos.getY() > 5.10 || currentPos.getY() < 0.01 || (currentPos.getY() > 1.4 && currentPos.getY() < 4.1) ){
            return errorRumbleControllerCommand();
        }

        //in the opening of the bottom entrance 
        if(currentPos.getY() > 0.01 && currentPos.getY() < 1.4){
            selectedEntrance = SelectedEntrance.BOTTOM;
        }

        //in the opening of the top entrance 
        if(currentPos.getY() > 4.1 && currentPos.getY() < 5.10){
            selectedEntrance = SelectedEntrance.TOP;
        }
        
        //if it slipped through everything somehow a catch all safety case
        if(selectedEntrance == SelectedEntrance.UNKNOWN){
            return errorRumbleControllerCommand();
        }

        return new SequentialCommandGroup(
            new FollowPathOnTheFly(selectedEntrance.insideCheckpointPos, drivetrain, drive_slow_test_vel, angular_slow_test_vel)
        );
    }

    //replace with better implementation of controller rumble 
    public Command errorRumbleControllerCommand(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.5)),
            Commands.waitSeconds(0.5),
            new InstantCommand(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0))
        );
    }

    public enum GridPositions {
        TOP_ENTRANCE_CHECKPOINT(0.0, 0.0, 0.0),
        BOTTOM_ENTRANCE_CHECKPOINT(2.17, 0.7, 180),
    
        GRID_8_LINEUP(2.0, 1.05, 180),
        GRID_8_SCORE(1.8, 1.05, 180);

        public final Pose2d pose;
    
        private GridPositions(double x, double y, double rotationDegrees) {
          pose = new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
        }
    }

    public static enum SelectedEntrance {
        TOP(GridPositions.TOP_ENTRANCE_CHECKPOINT.pose), 
        BOTTOM(GridPositions.BOTTOM_ENTRANCE_CHECKPOINT.pose),
        UNKNOWN(new Pose2d());

        public final Pose2d insideCheckpointPos;

        private SelectedEntrance(Pose2d pos){
            insideCheckpointPos = pos;
        }

    }
}
