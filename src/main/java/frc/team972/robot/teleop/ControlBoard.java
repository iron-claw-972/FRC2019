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


    public boolean getIntakeBall() { return mDriveControlBoard.getIntakeBall(); }
    public boolean getIntakeHatch() { return mDriveControlBoard.getIntakeHatch(); }
    public boolean getOuttake() { return mDriveControlBoard.getOuttake(); }
    public boolean getOuttakeReleased() { return mDriveControlBoard.getOuttakeReleased(); }
    public boolean getBallIntakeReleased() { return mDriveControlBoard.getBallIntakeReleased(); }



    public boolean getLevelOne() { return mDriveControlBoard.getLevelOne(); }
    public boolean getLevelTwo() { return mDriveControlBoard.getLevelTwo(); }
    public boolean getLevelThree() { return mDriveControlBoard.getLevelThree(); }


}
