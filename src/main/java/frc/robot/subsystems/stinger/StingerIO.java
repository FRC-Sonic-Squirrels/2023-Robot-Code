// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.stinger;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public interface StingerIO {

    // Contains all the input data received from hardware
    public static class StingerIOInputs implements LoggableInputs {

        public double StingerLengthInches = 0.0;
        public double StingerTargetLengthInches = 0.0;
        public double StingerVelocityInchesPerSecond = 0.0;
        public boolean StingerAtLowerLimit = false;
        public boolean StingerAtUpperLimit = false;
        public double StingerVelocityRPM = 0.0;
        public double StingerAppliedVolts = 0.0;
        public double[] StingerCurrentAmps = new double[] {};
        public double[] StingerTempCelsius = new double[] {};

        public void toLog(LogTable table) {
            table.put("StingerLengthInches", StingerLengthInches);
            table.put("StingerTargetLengthInches", StingerTargetLengthInches);
            table.put("StingerAtLowerLimit", StingerAtLowerLimit);
            table.put("StingerVelocityInchesPerSecond", StingerVelocityInchesPerSecond);
            table.put("StingerVelocityRPM", StingerVelocityRPM);
            table.put("StingerAppliedVolts", StingerAppliedVolts);
            table.put("StingerCurrentAmps", StingerCurrentAmps);
            table.put("StingerTempCelsius", StingerTempCelsius);
        }
        public void fromLog(LogTable table) {
            StingerAtLowerLimit = table.getBoolean("StingerAtLowerLimit", StingerAtLowerLimit);
            StingerAtUpperLimit = table.getBoolean("StingerAtUpperLimit", StingerAtUpperLimit);
            StingerVelocityInchesPerSecond = table.getDouble("StingerVelocityRPM", StingerVelocityRPM);
            StingerVelocityRPM = table.getDouble("StingerVelocityRPM", StingerVelocityRPM);
            StingerAppliedVolts = table.getDouble("StingerAppliedVolts", StingerAppliedVolts);
            StingerCurrentAmps = table.getDoubleArray("StingerCurrentAmps", StingerCurrentAmps);
            StingerTempCelsius = table.getDoubleArray("StingerTempCelsius", StingerTempCelsius);
        }
    }

    // updates the set of loggable inputs
    public default void updateInputs(StingerIOInputs inputs) {};

    /** Run the Stinger open loop at the specified voltage. */
    public default void setStingerVoltage(double volts) {}

    public default void setPercent(double percent) {}

    public default void setLengthInches(double targetLengthInches) {}

    public default void stop() {}

    public default void setMotionMagicSetPoint(double LengthInches) {}

    public default void hold() {}

    public default void setMotionMagicConstraints(double cruiseVelocity, double desiredTimeToSpeed) {}

    public default void zeroLength() {}
}
