package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private final LEDIO io;
  // no IOInputs because LEDs are a passive display, with no internal logic

  private colors currentColor;

  public LED(LEDIO io) {
    this.io = io;

    currentColor = colors.BLACK;

    // start with default color
    setColor(colors.NOTHING);
  }

  @Override
  public void periodic() {
    io.log();
  }

  public void setColor(colors color) {
    currentColor = color;
    io.setColor(color);
    SmartDashboard.putString("ledColor", currentColor.name());
  }

  public enum colors {
    RED(0.61),
    GREEN(0.77),
    BLUE(0.87),
    YELLOW(0.69),
    VIOLET(0.91),
    ORANGE(0.65),
    GOLD(0.67), // orange-ish
    BLACK(0.99),
    RED_STROBE(-0.11),
    BLUE_STROBE(-0.09),
    YELLOW_STROBE(-0.07),
    RAINBOW(-0.99),
    SCANNER_COLOR1(-0.01),
    SCANNER_COLOR2(0.19),
    WHITE_STROBE(-0.05),
    NOTHING(0.0);

    public final double colorValue;

    private colors(double colorValue) {
      this.colorValue = colorValue;
    }
  }
}
