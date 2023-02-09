package frc.robot.subsystems.streamdeck;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

public class StreamdeckData {
    
    public class Button {
        public String ButtonName = "Button";
        public String ButtonImage = "ButtonImage";
        public boolean Pressed = false;
        public boolean Targeted = false;
        public boolean Greyed = false;
        
        public Button(int x, int y, String buttonName) {
            this.set_buttonName("/streamdeck/" + buttonName);
            this.push_buttonStartup();
        }

        // Getters 
        public boolean get_pressedState() {
            return Pressed;
        }

        public boolean get_targetedState() {
            return Targeted;
        }

        public boolean get_greyedState() {
            return Greyed;
        }

        public String get_buttonName() {
            return ButtonName;
        }

        public String get_buttonImage() {
            return ButtonImage;
        }

        // Setters
        public void set_pressedState(boolean pressedState) {
            Pressed = pressedState;
        }

        public void set_targetedState(boolean targetedState) {
            Targeted = targetedState;
        }

        public void set_greyedState(boolean greyedState) {
            Greyed = greyedState;
        }

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
    }

    public Button[][] buttons = new Button[3][10];

    public StreamdeckData() {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 10; j++) {
                buttons[i][j] = new Button(i, j, String .valueOf(i) + "_" + String.valueOf(j));
            }
        }
    }

    public void printall(){
        Commands.print("Printing all buttons").schedule();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 10; j++) {
                Button button = getButton(i, j);
                Commands.print(button.get_buttonName() + " PRESSED: " + button.get_pressedState() + " TARGETED: " + button.get_targetedState() + " GREYED: " + button.get_greyedState()).schedule();
            }
        }
    }

    public Button getButton(int row, int column) {
        return buttons[row][column];
    }
}
