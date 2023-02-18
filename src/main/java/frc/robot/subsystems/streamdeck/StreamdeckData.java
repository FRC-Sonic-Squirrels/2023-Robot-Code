package frc.robot.subsystems.streamdeck;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * StreamdeckData, holds all the functions and data for the Streamdeck.
 *
 * <p>-
 *
 * <p>It Also contains the Button class, which is used to store button data and do operations on it.
 */
public class StreamdeckData {

  /**
   * Custom Button Class, used to store button data and do operations on it.
   *
   * <p>-
   *
   * <p>You can push, pull, and set data from the buttons to and from the SmartDashboard.
   */
  public class Button {
    /** The name of the button. <i>In the custom Button, its name is equal to its grid value.</i> */
    public String ButtonName = "Button";
    /** The image of the button. */
    public String ButtonImage = "ButtonImage";
    /** The pressed state of the button. */
    public boolean Pressed = false;
    /** The targeted state of the button. */
    public boolean Targeted = false;
    /** The greyed state of the button. */
    public boolean Greyed = false;

    /**
     * Button Constructor, used to create a new button.
     *
     * @param buttonName The name of the button. <i>In the custom Button, its name is equal to its
     *     grid value.</i>
     */
    public Button(String buttonName) {
      this.set_buttonName("/streamdeck/" + buttonName);
      this.push_buttonStartup();
    }

    // Getters

    /**
     * Gets the pressed state of the button.
     *
     * @return Returns if the button is pressed.
     */
    public boolean get_pressedState() {
      return Pressed;
    }

    /**
     * Gets the targeted state of the button.
     *
     * @return Returns if the button is targeted.
     */
    public boolean get_targetedState() {
      return Targeted;
    }

    /**
     * Gets the greyed state of the button.
     *
     * @return Returns if the button is greyed.
     */
    public boolean get_greyedState() {
      return Greyed;
    }

    /**
     * Gets the name of the button.
     *
     * @return Returns the name of the button.
     */
    public String get_buttonName() {
      return ButtonName;
    }

    /**
     * Gets the image of the button.
     *
     * @return Returns the image of the button.
     */
    public String get_buttonImage() {
      return ButtonImage;
    }

    // Setters

    /**
     * Sets the pressed state of the button.
     *
     * <p>-
     *
     * <p><b> !!! This function should only be used on startup for each button. !!! </b>
     *
     * @param pressedState The state of the button.
     */
    public void set_pressedState(boolean pressedState) {
      Pressed = pressedState;
      SmartDashboard.putBoolean(ButtonName, Pressed);
    }

    /**
     * Sets the targeted state of the button.
     *
     * @param targetedState The state of the button.
     */
    public void set_targetedState(boolean targetedState) {
      Targeted = targetedState;
      SmartDashboard.putBoolean(ButtonName + "Targeted", Targeted);
    }

    /**
     * Sets the greyed state of the button.
     *
     * @param greyedState The state of the button.
     */
    public void set_greyedState(boolean greyedState) {
      Greyed = greyedState;
      SmartDashboard.putBoolean(ButtonName + "Greyed", Greyed);
    }

    /**
     * Sets the name of the button.
     *
     * @param buttonName The name of the button.
     */
    public void set_buttonName(String buttonName) {
      ButtonName = buttonName;
    }

    public void set_buttonImage(String buttonImage) {
      ButtonImage = buttonImage;
    }

    public void push_buttonStartup() {
      SmartDashboard.putBoolean(ButtonName, Pressed);
      SmartDashboard.putBoolean(ButtonName + "Targeted", Targeted);
      SmartDashboard.putBoolean(ButtonName + "Greyed", Greyed);
      SmartDashboard.putString(ButtonName + "Image", ButtonImage);
    }

    public void push_targetstate() {
      SmartDashboard.putBoolean(ButtonName + "Targeted", Targeted);
    }
  }

  public Button[][] buttons = new Button[3][10];

  /**
   * Constructor for the StreamdeckData class. Creates all the buttons according to the grid size.
   */
  public StreamdeckData() {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 10; j++) {
        buttons[i][j] = new Button(i + "_" + j);
      }
    }
  }

  /** Prints all the buttons in the grid. */
  public void printall() {
    Commands.print("Printing all buttons").schedule();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 10; j++) {
        Button button = getButton(i, j);
        Commands.print(
                button.get_buttonName()
                    + " PRESSED: "
                    + button.get_pressedState()
                    + " TARGETED: "
                    + button.get_targetedState()
                    + " GREYED: "
                    + button.get_greyedState())
            .schedule();
      }
    }
  }

  public void push_targetstate_all() {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 10; j++) {
        Button button = getButton(i, j);
        button.get_targetedState();
        button.push_targetstate();
      }
    }
  }

  /**
   * Gets a button from the grid.
   *
   * @param row The row of the button.
   * @param column The column of the button.
   * @return Returns the button.
   */
  public Button getButton(int row, int column) {
    return buttons[row][column];
  }
}
