package frc.team972.robot.subsystems;

import frc.team972.robot.Constants;

public class SuperstructureSubsystem extends Subsystem {

    IntakeSubsystem intake_ = IntakeSubsystem.getInstance();

    enum State {
        UNSTOW_INTAKE, STOW_INTAKE, READY_INTAKE,
        SAFE_WRIST_INTAKE,
        WRIST_ELEVATOR_INTAKE_IDLE
    }

    private State state_;
    private State lastState_;

    private static SuperstructureSubsystem mInstance = new SuperstructureSubsystem();

    @Override
    public void fastPeriodic() {
        JointState intake_state = new JointState();
        JointState wrist_state = new JointState();

        State temp_state = state_;

        switch (state_) {
            //Moves the wrist/elevator assembly to a safe space for the wrist to move
            case SAFE_WRIST_INTAKE: {
                temp_state = lastState_;
                if(checkIntakeSafeMoveWrist(intake_state, wrist_state)) {
                    //move the wrist/elevator out of the way in the most efficent manner so that it doesn't hit the intake
                } else {
                    //return to the 'last state' before we needed to safe the wrist
                    temp_state = State.SAFE_WRIST_INTAKE;
                    state_ = lastState_;
                }
                break;
            }
            case UNSTOW_INTAKE: {
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state)) {
                    intake_.setIntake_goal_pos(Constants.kIntakeUnstowPosition);
                    if (checkTol(intake_state.angle, intake_state.unprofiled_goal_angle, Constants.kIntakeGeneralTolerance)) {
                        state_ = State.WRIST_ELEVATOR_INTAKE_IDLE;
                    }
                } else {
                    state_ = State.SAFE_WRIST_INTAKE;
                }
                break;
            }
            case STOW_INTAKE: {
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state)) {
                    intake_.setIntake_goal_pos(Constants.kIntakeStowPosition);
                    if (checkTol(intake_state.angle, intake_state.unprofiled_goal_angle, Constants.kIntakeGeneralTolerance)) {
                        state_ = State.WRIST_ELEVATOR_INTAKE_IDLE;
                    }
                } else {
                    state_ = State.SAFE_WRIST_INTAKE;
                }
                break;
            }
            case READY_INTAKE: {
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state)) {
                    intake_.setIntake_goal_pos(Constants.kIntakeReadyPosition);
                    if (checkTol(intake_state.angle, intake_state.unprofiled_goal_angle, Constants.kIntakeGeneralTolerance)) {
                        state_ = State.WRIST_ELEVATOR_INTAKE_IDLE;
                    }
                } else {
                    state_ = State.SAFE_WRIST_INTAKE;
                }
                break;
            }
        }

        lastState_ = temp_state;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }

    public boolean checkTol(double a, double b, double tol) {
        if (Math.abs(a - b) <= tol) {
            return true;
        } else {
            return false;
        }
    }

    public boolean checkIntakeSafeMoveWrist(JointState intake, JointState wrist) {
        return true; //TODO: intakes are not always safe to move...
    }

    public static SuperstructureSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new SuperstructureSubsystem();
        }
        return mInstance;
    }

}

class JointState {
    double angle;
    double angular_velocity;
    double goal_angle;
    double goal_angular_velocity;
    double unprofiled_goal_angle;
    double unprofiled_goal_angular_velocity;
}
