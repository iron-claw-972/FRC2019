package frc.team972.robot.teleop;

import frc.team972.robot.subsystems.Drive;
import frc.team972.robot.subsystems.ElevatorSubsystem;
import frc.team972.robot.util.MecanumHelper;

public class TeleopManager {
    private static TeleopManager mInstance = null;

    private Drive mDrive = Drive.getInstance();
    private ElevatorSubsystem mElevator = ElevatorSubsystem.getInstance();
    private ControlBoard controlBoard = ControlBoard.getInstance();

    public static TeleopManager getInstance() {
        if (mInstance == null) {
            mInstance = new TeleopManager();
        }
        return mInstance;
    }

    public void update() {
        mDrive.setOpenLoopMecanum(
                MecanumHelper.mecanumDrive(-controlBoard.getTranslateX(), controlBoard.getTranslateY(), controlBoard.getRotate(), controlBoard.getNoFieldOrient())
        );

        mElevator.readControllerInputs(
                controlBoard.getElevatorAxis()
        );
    }
}
