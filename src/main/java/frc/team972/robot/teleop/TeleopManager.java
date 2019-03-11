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
    private WristSubsystem mWrist = WristSubsystem.getInstance();

    private ControlBoard controlBoard = ControlBoard.getInstance();
    Joystick ghettoStick = new Joystick(1);

    public static TeleopManager getInstance() {
        if (mInstance == null) {
            mInstance = new TeleopManager();
        }
        return mInstance;
    }

    public void update() {

        mDrive.setMecanumDrivePoseDesired(null);
        mDrive.setCloseLoopMecanum(
                MecanumHelper.mecanumDrive(-controlBoard.getTranslateX(), controlBoard.getTranslateY(), controlBoard.getRotate(), controlBoard.getNoFieldOrient())
        );

        mWrist.setWrist_goal_pos(Math.toRadians(0));


        if(controlBoard.getTestButton()) {
            mElevator.setElevator_goal_pos(1.1);
        } else {
            mElevator.setElevator_goal_pos(0.0);
        }

        /*
        if(controlBoard.getTestButton()) {
            mWrist.setWrist_goal_pos(Math.toRadians(180-22));
        } else {
            mWrist.setWrist_goal_pos(Math.toRadians(90-22.0));
        }
        double roller_power = ghettoStick.getRawAxis(3) - ghettoStick.getRawAxis(2);
        mWrist.setRoller(-roller_power);
        */



    }
}
