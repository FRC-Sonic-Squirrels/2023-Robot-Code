// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.OverrideDrivetrainStop;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveAvoidBoxes extends CommandBase implements OverrideDrivetrainStop {
  private final Drivetrain drivetrain;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private boolean overideStopFlag = false;

  private static String ROOT_TABLE = "avoidance";

  /**
   * Create a new TeleopSwerve command object.
   *
   * @param drivetrain the drivetrain subsystem instructed by this command
   * @param translationXSupplier the supplier of the translation x value as a percentage of the
   *     maximum velocity
   * @param translationYSupplier the supplier of the translation y value as a percentage of the
   *     maximum velocity
   * @param rotationSupplier the supplier of the rotation value as a percentage of the maximum
   *     rotational velocity
   */
  TunableNumber constantMultiplier = new TunableNumber("avoidance/constantMultiplier", 0.1);

  TunableNumber exponent = new TunableNumber("avoidance/exponent", 3);

  TunableNumber distanceCutOff = new TunableNumber("avoidance/distanceCutoff", 0.6);

  BoundaryBoxes testBox = BoundaryBoxes.TEST_DEADZONE;

  // FIXME: REMOVE TESTDEADZONE
  BoundaryBoxes[] boundaryBoxes = {
    BoundaryBoxes.BLUE_CHARGING_PAD, BoundaryBoxes.BLUE_HUMAN_PLAYER /*BoundaryBoxes.TEST_DEADZONE*/
  };

  public DriveAvoidBoxes(
      Drivetrain drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drivetrain = drivetrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // BoundaryBoxes.TEST_DEADZONE.log();
    for (BoundaryBoxes boundaryBox : boundaryBoxes) {
      boundaryBox.log();
    }

    BoundaryBoxes.TEST_DEADZONE.log();
  }

  @Override
  public void execute() {

    // double startTime = System.currentTimeMillis();

    // invert the controller input and apply the deadband and squaring to make the robot more
    // responsive to small changes in the controller
    double xPercentage = -modifyAxis(translationXSupplier.getAsDouble());
    double yPercentage = -modifyAxis(translationYSupplier.getAsDouble());
    double rotationPercentage = -modifyAxis(rotationSupplier.getAsDouble());

    double xVelocity = xPercentage * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
    double yVelocity = yPercentage * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND;
    double rotationalVelocity =
        rotationPercentage * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", true);
    Logger.getInstance().recordOutput("avoidance/xDesired", xVelocity);
    Logger.getInstance().recordOutput("avoidance/yDesired", yVelocity);
    Logger.getInstance().recordOutput("avoidance/rotDesired", rotationalVelocity);

    double totalCorrectionX = 0.0;
    double totalCorrectionY = 0.0;

    for (BoundaryBoxes boundaryBox : boundaryBoxes) {
      for (int i = 0; i < boundaryBox.points.size(); i++) {
        var corrections =
            getCorrectionVelocities(
                drivetrain.getPose().getTranslation(),
                boundaryBox.points.get(i),
                new Translation2d(xVelocity, yVelocity),
                i);

        totalCorrectionX += corrections.getX();
        totalCorrectionY += corrections.getY();
      }
    }
    double finalX = xVelocity - totalCorrectionX;
    double finalY = yVelocity - totalCorrectionY;

    Logger logger = Logger.getInstance();

    // logger.recordOutput("avoidance/term1", normalizedVector.getX());

    // logger.recordOutput("avoidance/term2", (repulsionConstant / (distance * distance)));

    // logger.recordOutput("avoidance/robotVector", translationToPose(robotVector));
    // logger.recordOutput("avoidance/distance", distance);
    // logger.recordOutput("avoidance/normalizedVector", translationToPose(normalizedVector));
    // logger.recordOutput("avoidance/correctionX", correctionX);
    // logger.recordOutput("avoidance/correctionY", correctionY);
    // logger.recordOutput("avoidance/finalX", finalX);
    // logger.recordOutput("avoidance/finalY", finalY);

    // logger.recordOutput("avoidance/point", translationToPose(point));

    logger.recordOutput(
        "avoidance/desiredVector", translationToPose(new Translation2d(xVelocity, yVelocity)));
    // logger.recordOutput("avoidance/dotProduct", dotProduct);

    // if (dotProduct > 0) {
    //   drivetrain.drive(finalX, finalY, rotationalVelocity);
    // } else {
    //   drivetrain.drive(xVelocity, yVelocity, rotationalVelocity);
    // }

    // var timeTaken = System.currentTimeMillis() - startTime;
    // logger.recordOutput("avoidance/timeTakenInExecute", timeTaken);
    // System.out.println("TIME TAKEN IN EXECUTE: " + timeTaken);
    drivetrain.drive(finalX, finalY, rotationalVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    if (overideStopFlag) {
      overideStopFlag = false;
      // Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", false);
      return;
    }

    this.drivetrain.stop();

    // super.end(interrupted);

    // Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", false);
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value
   * @return
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, DrivetrainConstants.DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  @Override
  public void overideStop() {
    overideStopFlag = true;
  }

  public static double FIELD_LENGTH_HALF_WAY = 8.3;
  public static double FIELD_WIDTH_HALF_WAY = 8.02 / 2;
  public static double FIELD_WIDTH = 8.02;

  Translation2d point0 = new Translation2d(3.3, FIELD_WIDTH - 3.92);
  Translation2d point1 = new Translation2d(point0.getX(), FIELD_WIDTH - 5.5);

  Translation2d point2 = new Translation2d(point0.getX() + 1.6, FIELD_WIDTH - 3.92);
  Translation2d point3 = new Translation2d(point2.getX(), FIELD_WIDTH - 5.5);

  Translation2d[] points = {point0, point1, point2, point3};

  public Translation2d robotToPointVector(Translation2d currentPose, Translation2d pointPose) {
    // Translation2d robotVector;

    var xDiff = pointPose.getX() - currentPose.getX();
    var yDiff = pointPose.getY() - currentPose.getY();

    return new Translation2d(xDiff, yDiff);
  }

  public double getDistance(Translation2d vector) {
    return new Translation2d().getDistance(vector);
  }

  public Translation2d normalize(Translation2d vector) {
    var distance = getDistance(vector);

    return new Translation2d(vector.getX() / distance, vector.getY() / distance);
  }

  public Pose2d translationToPose(Translation2d translation) {
    return new Pose2d(translation, translation.getAngle());
  }

  public double dotProduct(Translation2d first, Translation2d second) {
    return (first.getX() * second.getX()) + (first.getY() * second.getY());
  }

  public Translation2d getCorrectionVelocities(
      Translation2d currentPose,
      Translation2d pointPose,
      Translation2d desiredVector,
      int pointNumber) {

    var robotVector = robotToPointVector(drivetrain.getPose().getTranslation(), pointPose);
    var distance = getDistance(robotVector);

    if (distance > distanceCutOff.get()) {
      return new Translation2d();
    }

    var normalizedVector = normalize(robotVector);

    double correctionX;
    double correctionY;

    double repulsionConstant =
        DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND * constantMultiplier.get();

    correctionX =
        (normalizedVector.getX() * (repulsionConstant / (Math.pow(distance, exponent.get()))));
    correctionY =
        (normalizedVector.getY() * (repulsionConstant / (Math.pow(distance, exponent.get()))));

    // var logger = Logger.getInstance();
    // logger.recordOutput("avoidance/pointPose" + pointNumber, translationToPose(pointPose));
    // logger.recordOutput("avoidance/robotVector" + pointNumber, translationToPose(robotVector));
    // logger.recordOutput("avoidance/distance" + pointNumber, distance);
    // logger.recordOutput(
    //     "avoidance/normalizedVector" + pointNumber, translationToPose(normalizedVector));
    // logger.recordOutput("avoidance/correctionX" + pointNumber, correctionX);
    // logger.recordOutput("avoidance/correctionY" + pointNumber, correctionY);

    // var desiredVector = new Translation2d(xVelocity, yVelocity);

    double dotProduct = dotProduct(desiredVector, robotVector);

    // logger.recordOutput("avoidance/dotProduct" + pointNumber, dotProduct);

    if (dotProduct > 0) {
      return new Translation2d(correctionX, correctionY);
    }

    return new Translation2d();
  }

  private enum BoundaryBoxes {
    TEST_DEADZONE(new Translation2d(7.5, 5), new Translation2d(10, 2.5), 0.1),

    BLUE_CHARGING_PAD(new Translation2d(2.95, 4.0), new Translation2d(4.9, 1.5), 0.1),

    BLUE_HUMAN_PLAYER(new Translation2d(1.4, 7), new Translation2d(3.35, 5.55), 0.1);

    private Translation2d topL;
    private Translation2d topR;
    private Translation2d backL;
    private Translation2d backR;
    private double spacing;

    public final ArrayList<Translation2d> points = new ArrayList<Translation2d>();

    private BoundaryBoxes(Translation2d backL, Translation2d topR, double spacingBetweenPoints) {
      this.backL = backL;
      this.topR = topR;

      this.topL = new Translation2d(topR.getX(), backL.getY());
      this.backR = new Translation2d(backL.getX(), topR.getY());

      spacing = spacingBetweenPoints;

      points.add(backL);
      points.add(backR);
      points.add(topL);
      points.add(topR);

      generatePointsForHorizontalEdge(backL, topL);
      generatePointsForHorizontalEdge(backR, topR);

      generatePointsForVerticalEdge(backR, backL);
      generatePointsForVerticalEdge(topR, topL);
    }

    private void generatePointsForHorizontalEdge(Translation2d left, Translation2d right) {
      // var middle = right.getX() - left.getX();

      // double yConstant = left.getY();
      // double currentX = left.getX() + spacing;
      // while (Math.abs(currentX - right.getX()) > spacing) {
      //   points.add(new Translation2d(currentX, yConstant));
      //   currentX += spacing;
      // }
      var leftPointer = left.getX() + spacing;
      var rightPointer = right.getX() - spacing;

      var constantY = left.getY();

      while ((rightPointer - leftPointer) >= spacing * 2) {
        points.add(new Translation2d(leftPointer, constantY));
        points.add(new Translation2d(rightPointer, constantY));

        leftPointer += spacing;
        rightPointer -= spacing;
      }

      var middleX = (leftPointer + rightPointer) / 2;

      points.add(new Translation2d(middleX, constantY));
    }

    private void generatePointsForVerticalEdge(Translation2d bottom, Translation2d top) {
      // var middle = right.getX() - left.getX();

      // double xConstant = bottom.getX();
      // double currentY = bottom.getY() + spacing;

      // while (Math.abs(currentY - top.getY()) > spacing) {
      //   points.add(new Translation2d(xConstant, currentY));
      //   currentY += spacing;
      // }

      var bottomPointer = bottom.getY() + spacing;
      var topPointer = top.getY() - spacing;

      var constantX = bottom.getX();

      System.out.println("top - bottom" + (topPointer - bottomPointer));
      System.out.println("space2X" + spacing * 2);

      while ((topPointer - bottomPointer) >= spacing * 2) {
        points.add(new Translation2d(constantX, bottomPointer));
        points.add(new Translation2d(constantX, topPointer));

        bottomPointer += spacing;
        topPointer -= spacing;
      }

      var middleY = (bottomPointer + topPointer) / 2;

      points.add(new Translation2d(constantX, middleY));
    }

    public void log() {
      var logger = Logger.getInstance();

      for (int i = 0; i < points.size(); i++) {
        logger.recordOutput(
            ROOT_TABLE + "/boundaryBoxes/" + this.name() + "/" + i, transToPos0Rot(points.get(i)));
      }

      ArrayList<State> states = new ArrayList<State>();
      states.add(createState(topL));
      states.add(createState(topR));
      states.add(createState(backR));
      states.add(createState(backL));
      states.add(createState(topL));

      Trajectory traj = new Trajectory(states);

      logger.recordOutput(ROOT_TABLE + "/boundaryBoxes/" + this.name() + "/traj", traj);
    }

    private State createState(Translation2d loc) {
      return new State(0, 0, 0, new Pose2d(loc, new Rotation2d()), 0);
    }

    private Pose2d transToPos0Rot(Translation2d trans) {
      return new Pose2d(trans, new Rotation2d());
    }
  }
}
