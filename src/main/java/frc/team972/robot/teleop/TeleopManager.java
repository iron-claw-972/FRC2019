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
    Joystick stick = new Joystick(1);

    public static TeleopManager getInstance() {
        if (mInstance == null) {
            mInstance = new TeleopManager();
        }
        return mInstance;
    }

    public void update() {


        if (controlBoard.getTestButton()) {
            IntakeSubsystem.getInstance().setIntake_goal_pos(Math.toRadians(80));

            //mDrive.setMecanumDrivePoseDesired(new Pose2d(1, 1, Rotation2d.fromDegrees(0)));
        } else {
            IntakeSubsystem.getInstance().setIntake_goal_pos(Math.toRadians(10));

            /*
            mDrive.setMecanumDrivePoseDesired(null);
            mDrive.setCloseLoopMecanum(
                    MecanumHelper.mecanumDrive(-controlBoard.getTranslateX(), controlBoard.getTranslateY(), controlBoard.getRotate(), controlBoard.getNoFieldOrient())
            );
            */
        }

        mDrive.setOpenLoopMecanum(
                MecanumHelper.mecanumDrive(-controlBoard.getTranslateX(), controlBoard.getTranslateY(), controlBoard.getRotate(), controlBoard.getNoFieldOrient())
        );

        IntakeSubsystem.getInstance().setRollerPower(-Math.abs(stick.getRawAxis(3)));

        //mExample.setDesiredVoltage(controlBoard.getExampleJoystickValue());

        /*
        double wrist_goal = 0;
        if(stick.getRawButton(6)) {
            wrist_goal = Math.PI;
        } else if(stick.getRawButton(5)) {
            wrist_goal = Math.PI / 2;
        }

        WristSubsystem.getInstance().setWrist_goal_pos(wrist_goal);
        */
//        mElevator.getElevatorController().SetGoal(1);


        /*
        if(controlBoard.getTestButton()) {
            mWrist.setWrist_goal_pos(Math.toRadians(90));
        } else if(controlBoard.getTestButton2()) {
            mWrist.setWrist_goal_pos(Math.toRadians(180));
        }
        else {
            mWrist.setWrist_goal_pos(Math.toRadians(0));
        }
        */

        //mWrist.setWrist_goal_pos(Math.toRadians(stick.getRawAxis(3) * (180 + 45)));

        //mWrist.setRoller(stick.getRawAxis(2));

    }
}
