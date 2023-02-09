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
        
        public Button(int x, int y) {
            this.set_buttonName(x + "_" + y);
            this.update_buttonName("push");
            this.update_buttonImage("push");
            this.update_pressedState("push");
            this.update_targetedState("push");
            this.update_greyedState("push");
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


        // Updater
        public void update_pressedState(String mode) {
            if (mode == "push"){
                SmartDashboard.putBoolean("Button" + ButtonName + "_Pressed", Pressed);
            }
            else if (mode == "pull"){
                Pressed = SmartDashboard.getBoolean("Button" + ButtonName + "_Pressed", false);
            }
        }
        
        public void update_targetedState(String mode) {
            if (mode == "push"){
                SmartDashboard.putBoolean("Button" + ButtonName + "_Targeted", Targeted);
            }
            else if (mode == "pull"){
                Targeted = SmartDashboard.getBoolean("Button" + ButtonName + "_Targeted", false);
            }
        }

        public void update_greyedState(String mode) {
            if (mode == "push"){
                SmartDashboard.putBoolean("Button" + ButtonName + "_Greyed", Greyed);
            }
            else if (mode == "pull"){
                Greyed = SmartDashboard.getBoolean("Button" + ButtonName + "_Greyed", false);
            }
        }

        public void update_buttonName(String mode) {
            if (mode == "push"){
                SmartDashboard.putString("Button" + ButtonName + "_Name", ButtonName);
            }
            else if (mode == "pull"){
                ButtonName = SmartDashboard.getString("Button" + ButtonName + "_Name", "Button");
            }
        }

        public void update_buttonImage(String mode) {
            if (mode == "push"){
                SmartDashboard.putString("Button" + ButtonName + "_Image", ButtonImage);
            }
            else if (mode == "pull"){
                ButtonImage = SmartDashboard.getString("Button" + ButtonName + "_Image", "ButtonImage");
            }
        }

        // Other
        public void pushloop() {
            update_targetedState("push");
            update_greyedState("push");
            update_buttonImage("push");
        }

        public void pullloop() {
            update_pressedState("pull");
        }
    }

    public Button[][] buttons = new Button[3][10];

    public StreamdeckData() {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 10; j++) {
                buttons[i][j] = new Button(i, j);
            }
        }
    }

    public void pushloop() {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 10; j++) {
                Button button = getButton(i, j);
                button.pushloop();
            }
        }
    }

    public void pullloop() {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 10; j++) {
                Button button = getButton(i, j);
                button.pullloop();
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
