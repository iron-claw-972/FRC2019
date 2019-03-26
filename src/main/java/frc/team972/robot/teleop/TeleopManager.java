package frc.team972.robot.teleop;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import frc.team972.robot.lib.Pose2d;
import frc.team972.robot.lib.Rotation2d;
import frc.team972.robot.statemachines.SuperstructureState;
import frc.team972.robot.subsystems.*;

import frc.team972.robot.util.MecanumHelper;

public class TeleopManager {
    private static TeleopManager mInstance = null;

    DoubleSolenoid climbA = new DoubleSolenoid(40, 0, 1);
    DoubleSolenoid climbB = new DoubleSolenoid(40, 6, 7);

    private DriveSubsystem mDrive = DriveSubsystem.getInstance();
    private SuperstructureSubsystem mSuperstructure = SuperstructureSubsystem.getInstance();

    private ControlBoard controlBoard = ControlBoard.getInstance();

    public static TeleopManager getInstance() {
        if (mInstance == null) {
            mInstance = new TeleopManager();
        }
        return mInstance;
    }

    public void update() {

        mDrive.setMecanumDrivePoseDesired(null);
        mDrive.setCloseLoopMecanum(
                MecanumHelper.mecanumDrive(-controlBoard.getTranslateX(), controlBoard.getTranslateY(), controlBoard.getRotate(), controlBoard.getA())
        );

        SuperstructureState superstructureState = mSuperstructure.getState();
        if (controlBoard.getIntakeStow()) {
            mSuperstructure.setState(SuperstructureState.STOW_WRIST);
        } else if (controlBoard.getIntakeBall()) {
            mSuperstructure.setState(SuperstructureState.INTAKE_BALL_WRIST_FLAT);
        } else if (controlBoard.getIntakeHatch()) {
            mSuperstructure.setState(SuperstructureState.PREPARE_HATCH_INTAKE);
        } else if (controlBoard.getIntakeBallLip()) {
            mSuperstructure.setState(SuperstructureState.INTAKE_BALL_WRIST_LIP);
        } else if (controlBoard.getLevelOne()) {
            if (superstructureState == SuperstructureState.READY_WRIST_RAISE) {
                mSuperstructure.setState(SuperstructureState.READY_BALL_LEVEL_1);
            } else if (superstructureState == SuperstructureState.READY_WRIST_LIP) {
                mSuperstructure.setState(SuperstructureState.READY_HATCH_LEVEL_1);
            } else if ((superstructureState == SuperstructureState.READY_BALL_LEVEL_2) || (superstructureState == SuperstructureState.READY_BALL_LEVEL_3)) {
                mSuperstructure.setState(SuperstructureState.READY_BALL_LEVEL_1);
            } else if ((superstructureState == SuperstructureState.READY_HATCH_LEVEL_2) || (superstructureState == SuperstructureState.READY_HATCH_LEVEL_3)) {
                mSuperstructure.setState(SuperstructureState.READY_HATCH_LEVEL_1);
            }
        } else if (controlBoard.getLevelTwo()) {
            if (superstructureState == SuperstructureState.READY_WRIST_RAISE) {
                mSuperstructure.setState(SuperstructureState.READY_BALL_LEVEL_2);
            } else if (superstructureState == SuperstructureState.READY_WRIST_LIP) {
                mSuperstructure.setState(SuperstructureState.READY_HATCH_LEVEL_2);
            } else if ((superstructureState == SuperstructureState.READY_BALL_LEVEL_1) || (superstructureState == SuperstructureState.READY_BALL_LEVEL_3)) {
                mSuperstructure.setState(SuperstructureState.READY_BALL_LEVEL_2);
            } else if ((superstructureState == SuperstructureState.READY_HATCH_LEVEL_1) || (superstructureState == SuperstructureState.READY_HATCH_LEVEL_3)) {
                mSuperstructure.setState(SuperstructureState.READY_HATCH_LEVEL_2);
            }
        } else if (controlBoard.getLevelThree()) {
            if (superstructureState == SuperstructureState.READY_WRIST_RAISE) {
                mSuperstructure.setState(SuperstructureState.READY_BALL_LEVEL_3);
            } else if (superstructureState == SuperstructureState.READY_WRIST_LIP) {
                mSuperstructure.setState(SuperstructureState.READY_HATCH_LEVEL_3);
            } else if ((superstructureState == SuperstructureState.READY_BALL_LEVEL_1) || (superstructureState == SuperstructureState.READY_BALL_LEVEL_2)) {
                mSuperstructure.setState(SuperstructureState.READY_BALL_LEVEL_3);
            } else if ((superstructureState == SuperstructureState.READY_HATCH_LEVEL_1) || (superstructureState == SuperstructureState.READY_HATCH_LEVEL_2)) {
                mSuperstructure.setState(SuperstructureState.READY_HATCH_LEVEL_3);
            }
        }

        /*
        if(+) {
            climbA.set(DoubleSolenoid.Value.kForward);
        } else {
            climbA.set(DoubleSolenoid.Value.kReverse);
        }
        */

    }
}
