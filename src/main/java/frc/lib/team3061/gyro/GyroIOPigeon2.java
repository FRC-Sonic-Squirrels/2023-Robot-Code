/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 gyro;
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon2(int id, String canBusName) {
    gyro = new Pigeon2(id, canBusName);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    gyro.getRawGyro(xyzDps);
    inputs.connected = gyro.getLastError().equals(ErrorCode.OK);
    inputs.positionDeg = gyro.getYaw(); // degrees
    inputs.velocityDegPerSec = xyzDps[2]; // degrees per second
  }
}
