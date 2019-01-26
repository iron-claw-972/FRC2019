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
    private ElevatorJoystickControlBoard mElevatorControlBoard;

    private ControlBoard() {
        mDriveControlBoard = GamepadDriveControlBoard.getInstance();
        mElevatorControlBoard = ElevatorJoystickControlBoard.getInstance();
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

    public double getElevatorAxis() { return mElevatorControlBoard.getElevatorAxis(); }
}
