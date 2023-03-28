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
    HOT_PINK(0.57),
    DARK_RED(0.59),
    RED_ORANGE(0.63),
    LAWN_GREEN(0.71),
    LIME(0.73),
    DARK_GREEN(0.75),
    BLUE_GREEN(0.79),
    AQUA(0.81),
    SKY_BLUE(0.83),
    DARK_BLUE(0.85),
    BLUE_VIOLET(0.89),
    WHITE(0.93),
    GRAY(0.95),
    DARK_GRAY(0.97),
    RED_STROBE(-0.11),
    BLUE_STROBE(-0.09),
    YELLOW_STROBE(-0.07),
    BREATH_RED(-0.17),
    BREATH_BLUE(-0.15),
    LIGHT_CHASE_RED(-0.31),
    LIGHT_CHASE_BLUE(-0.29),
    RAINBOW_TWINKLES(-0.55),
    RAINBOW_COLOR_WAVES(-0.45),
    CONFETTI(-0.87),
    FIRE_LARGE(-0.57),
    RAINBOW(-0.99),
    SCANNER_COLOR1(-0.01),
    SCANNER_COLOR2(0.19),
    //
    NOTHING(0.0);

    public final double colorValue;

    private colors(double colorValue) {
      this.colorValue = colorValue;
    }
  }
}
