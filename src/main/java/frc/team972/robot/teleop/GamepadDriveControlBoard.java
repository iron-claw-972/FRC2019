package frc.team972.robot.teleop;

import edu.wpi.first.wpilibj.Joystick;
import frc.team972.robot.Constants;

public class GamepadDriveControlBoard {
    private static GamepadDriveControlBoard mInstance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDriveControlBoard();
        }

        return mInstance;
    }

    private Joystick mJoystick;

    private GamepadDriveControlBoard() {
        mJoystick = new Joystick(Constants.kDriveGamepadPort);
    }

    public double getTranslateY() {
        //Up and Down on the Left Joystick
        return -mJoystick.getRawAxis(1);
    }

    public double getTranslateX() {
        //Left and Right on the Left Joystick
        return mJoystick.getRawAxis(0);
    }

    public double getRotate() {
        //Left and Right on the Right Joystick
        //TODO: Fill Axis!
        return -mJoystick.getRawAxis(4);
    }

    public boolean getTestButton() {
        return mJoystick.getRawButton(1);
    }

    public boolean getTestButton2() {
        return mJoystick.getRawButton(2);
    }


    public boolean getNoFieldOrient() {
        return mJoystick.getRawButton(6);
    }

}