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


    public boolean getIntakeBall() { return false; }
    public boolean getIntakeHatch() { return false; }
    public boolean getOuttake() { return false; }

    public boolean getLevelOne() { return false; }
    public boolean getLevelTwo() { return false; }
    public boolean getLevelThree() { return false; }


}
