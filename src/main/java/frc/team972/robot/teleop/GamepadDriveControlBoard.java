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
    private Joystick stickDrive = new Joystick(0);

    private GamepadDriveControlBoard() {
        mJoystick = new Joystick(Constants.kDriveGamepadPort);
    }

    public double getTranslateY() {
        //Up and Down on the Left Joystick
        return -stickDrive.getRawAxis(1);
    }

    public double getTranslateX() {
        //Left and Right on the Left Joystick
        return stickDrive.getRawAxis(0);
    }

    public double getRotate() {
        //Left and Right on the Right Joystick
        //TODO: Fill Axis!
        return -stickDrive.getRawAxis(4);
    }

    public boolean getA() {
        return (stickDrive.getRawAxis(3) > 0.5);
    }

    public boolean getB() {
        return (stickDrive.getRawAxis(2) > 0.5);
    }


    public boolean getIntakeStow() { return mJoystick.getRawAxis(2) > 0.5; }
    public boolean getIntakeBall() { return mJoystick.getRawButton(6); }
    public boolean getIntakeHatch() { return mJoystick.getRawAxis(3) > 0.25; }
    public boolean getOuttake() { return mJoystick.getRawButton(5); }
    public boolean getOuttakeReleased() { return mJoystick.getRawButtonReleased(5); }
    public boolean getBallIntakeReleased() { return mJoystick.getRawButtonReleased(6); }

    public boolean getIntakeBallLip() { return mJoystick.getRawButton(3); }

    public boolean getLevelOne() { return mJoystick.getRawButton(1); }
    public boolean getLevelTwo() { return mJoystick.getRawButton(2); }
    public boolean getLevelThree() { return mJoystick.getRawButton(4); }


    public boolean getNoFieldOrient() {
        return stickDrive.getRawButton(1);
    }

}