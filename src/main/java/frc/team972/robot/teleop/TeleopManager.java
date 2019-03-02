package frc.team972.robot.teleop;

import edu.wpi.first.wpilibj.Joystick;
import frc.team972.robot.lib.Pose2d;
import frc.team972.robot.lib.Rotation2d;
import frc.team972.robot.subsystems.*;

import frc.team972.robot.util.MecanumHelper;

public class TeleopManager {
    private static TeleopManager mInstance = null;

    private DriveSubsystem mDrive = DriveSubsystem.getInstance();
    private ElevatorSubsystem mElevator = ElevatorSubsystem.getInstance();
    private ExampleSubsystem mExample = ExampleSubsystem.getInstance();
    private WristSubsystem mWrist = WristSubsystem.getInstance();

    private ControlBoard controlBoard = ControlBoard.getInstance();

    public static TeleopManager getInstance() {
        if (mInstance == null) {
            mInstance = new TeleopManager();
        }
        return mInstance;
    }

    public void update() {

        if(controlBoard.getTestButton()) {
            mDrive.setMecanumDrivePoseDesired(new Pose2d(0,0, Rotation2d.fromDegrees(0))); //go to origin
        } else {
            mDrive.setMecanumDrivePoseDesired(null);
            mDrive.setCloseLoopMecanum(
                    MecanumHelper.mecanumDrive(-controlBoard.getTranslateX(), controlBoard.getTranslateY(), controlBoard.getRotate(), controlBoard.getNoFieldOrient())
            );
        }
    }
}
