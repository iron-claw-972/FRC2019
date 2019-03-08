package frc.team972.robot.teleop;

public class ControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private GamepadDriveControlBoard mDriveControlBoard;

    private ControlBoard() {
        mDriveControlBoard = GamepadDriveControlBoard.getInstance();
    }

    public double getTranslateX() {
        return mDriveControlBoard.getTranslateX();
    }

    public double getTranslateY() {
        return mDriveControlBoard.getTranslateY();
    }

    public double getRotate() {
        return mDriveControlBoard.getRotate();
    }

    public boolean getNoFieldOrient() {
        return mDriveControlBoard.getNoFieldOrient();
    }

    public boolean getTestButton() {
        return mDriveControlBoard.getTestButton();
    }

    public boolean getTestButtonPressed() {
        return mDriveControlBoard.getTestButtonPressed();
    }

    public boolean getTestButton2() {
        return mDriveControlBoard.getTestButton2();
    }

    public double getExampleJoystickValue() {
        return 0.254;
    }

}
