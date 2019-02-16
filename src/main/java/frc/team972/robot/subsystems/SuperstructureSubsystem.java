package frc.team972.robot.subsystems;

import frc.team972.robot.Constants;

public class SuperstructureSubsystem extends Subsystem {

    IntakeSubsystem intake_ = IntakeSubsystem.getInstance();

    enum State {
        REQUEST_INTAKE_UNSTOW, REQUEST_INTAKE_READY, REQUEST_INTAKE_STOW,
        STOW_INTAKE,
        UNSTOW_INTAKE_FROM_STOW, UNSTOW_INTAKE_FROM_READY,
        READY_INTAKE_FROM_STOW, READY_INTAKE_FROM_UNSTOW,
        MOVE_ELEVATOR_WRST_FOR_INTAKE_UNSTOW_SAFE, MOVE_ELEVATOR_WRST_FOR_INTAKE_READY_SAFE, MOVE_ELEVATOR_WRST_FOR_INTAKE_STOW_SAFE,
        CNC_CRASH,
        WRIST_ELEVATOR_IDLE_INTAKE_UNSTOW, WRIST_ELEVATOR_IDLE_INTAKE_STOW, WRIST_ELEVATOR_IDLE_INTAKE_READY
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
            case CNC_CRASH: {
                System.out.println("CNC_CRASH . . . fix this later?");
                break;
            }

            case REQUEST_INTAKE_STOW: {
                if (lastState_ == State.WRIST_ELEVATOR_IDLE_INTAKE_UNSTOW) {
                    System.out.println("redundant action");
                    state_ = lastState_;
                } else if (lastState_ == State.WRIST_ELEVATOR_IDLE_INTAKE_UNSTOW) {
                    state_ = State.STOW_INTAKE;
                } else if (lastState_ == State.WRIST_ELEVATOR_IDLE_INTAKE_READY) {
                    state_ = State.STOW_INTAKE;
                }
                break;
            }

            case REQUEST_INTAKE_UNSTOW: {
                if (lastState_ == State.WRIST_ELEVATOR_IDLE_INTAKE_STOW) {
                    state_ = State.UNSTOW_INTAKE_FROM_STOW;
                } else if (lastState_ == State.WRIST_ELEVATOR_IDLE_INTAKE_UNSTOW) {
                    System.out.println("redundant action");
                    state_ = lastState_;
                } else if (lastState_ == State.WRIST_ELEVATOR_IDLE_INTAKE_READY) {
                    state_ = State.UNSTOW_INTAKE_FROM_READY;
                }
                break;
            }

            case REQUEST_INTAKE_READY: {
                if (lastState_ == State.WRIST_ELEVATOR_IDLE_INTAKE_STOW) {
                    state_ = lastState_.READY_INTAKE_FROM_STOW;
                } else if (state_ == State.WRIST_ELEVATOR_IDLE_INTAKE_UNSTOW) {
                    state_ = State.READY_INTAKE_FROM_UNSTOW;
                } else if (lastState_ == State.WRIST_ELEVATOR_IDLE_INTAKE_READY) {
                    System.out.println("redundant action");
                    state_ = lastState_;
                }
                break;
            }

            // IDLES ------------------
            case WRIST_ELEVATOR_IDLE_INTAKE_STOW: {

                break;
            }
            case WRIST_ELEVATOR_IDLE_INTAKE_UNSTOW: {

                break;
            }
            case WRIST_ELEVATOR_IDLE_INTAKE_READY: {

                break;
            }
            // ------------------

            case STOW_INTAKE: {
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state)) {
                    intake_.setIntake_goal_pos(Constants.kIntakeStowPosition);
                    if (checkTol(intake_state.angle, intake_state.unprofiled_goal_angle, Constants.kIntakeGeneralTolerance)) {
                        state_ = State.WRIST_ELEVATOR_IDLE_INTAKE_STOW;
                    }
                } else if (lastState_ != State.STOW_INTAKE) {
                    state_ = State.MOVE_ELEVATOR_WRST_FOR_INTAKE_STOW_SAFE;
                } else {
                    state_ = State.CNC_CRASH;
                }
                break;
            }

            case UNSTOW_INTAKE_FROM_READY: {
                intake_.setIntake_goal_pos(Constants.kIntakeUnstowPosition);
                if (checkTol(intake_state.angle, intake_state.unprofiled_goal_angle, Constants.kIntakeGeneralTolerance)) {
                    state_ = State.WRIST_ELEVATOR_IDLE_INTAKE_UNSTOW;
                }
                break;
            }

            case UNSTOW_INTAKE_FROM_STOW: {
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state)) {
                    intake_.setIntake_goal_pos(Constants.kIntakeUnstowPosition);
                    if (checkTol(intake_state.angle, intake_state.unprofiled_goal_angle, Constants.kIntakeGeneralTolerance)) {
                        state_ = State.WRIST_ELEVATOR_IDLE_INTAKE_UNSTOW;
                    }
                } else if (lastState_ != State.UNSTOW_INTAKE_FROM_STOW) {
                    state_ = State.MOVE_ELEVATOR_WRST_FOR_INTAKE_UNSTOW_SAFE;
                } else {
                    state_ = State.CNC_CRASH;
                }
                break;
            }

            case READY_INTAKE_FROM_STOW: {
                intake_.setIntake_goal_pos(Constants.kIntakeReadyPosition);
                if (checkTol(intake_state.angle, intake_state.unprofiled_goal_angle, Constants.kIntakeGeneralTolerance)) {
                    state_ = State.WRIST_ELEVATOR_IDLE_INTAKE_READY;
                }
                break;
            }

            case READY_INTAKE_FROM_UNSTOW: {
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state)) {
                    intake_.setIntake_goal_pos(Constants.kIntakeReadyPosition);
                    if (checkTol(intake_state.angle, intake_state.unprofiled_goal_angle, Constants.kIntakeGeneralTolerance)) {
                        state_ = State.WRIST_ELEVATOR_IDLE_INTAKE_READY;
                    }
                } else if (lastState_ != State.READY_INTAKE_FROM_UNSTOW) {
                    state_ = State.MOVE_ELEVATOR_WRST_FOR_INTAKE_READY_SAFE;
                } else {
                    state_ = State.CNC_CRASH;
                }
                break;
            }

            // -----------

            case MOVE_ELEVATOR_WRST_FOR_INTAKE_UNSTOW_SAFE: {
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state)) {
                    state_ = State.UNSTOW_INTAKE_FROM_STOW;
                } else {
                    //TODO: move stuff
                }
                break;
            }

            case MOVE_ELEVATOR_WRST_FOR_INTAKE_READY_SAFE: {
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state)) {
                    state_ = State.READY_INTAKE_FROM_STOW;
                } else {
                    //TODO: move stuff
                }
                break;
            }

            case MOVE_ELEVATOR_WRST_FOR_INTAKE_STOW_SAFE: {
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state)) {
                    state_ = State.STOW_INTAKE;
                } else {
                    //TODO: move stuff
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
