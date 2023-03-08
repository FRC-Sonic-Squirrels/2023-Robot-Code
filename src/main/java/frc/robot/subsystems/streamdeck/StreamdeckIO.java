package frc.robot.subsystems.streamdeck;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.driverassist.LogicalGridLocation;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface StreamdeckIO {

  Trigger getButton(String buttonNumber);

  boolean getPressed(String buttonNumber);

  void setButton(String buttonNumber, boolean value);

  void setDesiredFromButton(String buttonNumber);


  LogicalGridLocation getDesiredFromButton(String buttonNumber);


  boolean[][] getButtonsPressed(boolean[][] ButtonGrid);

  boolean ifControlButtonsPressed();

  String getControlButtonPressed();

  boolean ifTargetingButtonsValid();

  String getTargetingButtonPressed();

  void init();

  class StreamdeckInputs implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
      // TODO Auto-generated method stub

    }

    @Override
    public void fromLog(LogTable table) {
      // TODO Auto-generated method stub

    }
  }
}
