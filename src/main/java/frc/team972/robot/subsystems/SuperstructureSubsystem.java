package frc.team972.robot.subsystems;

import frc.team972.robot.Constants;

public class SuperstructureSubsystem extends Subsystem {

    IntakeSubsystem intake_ = IntakeSubsystem.getInstance();
    WristSubsystem wrist_ = WristSubsystem.getInstance();
    ElevatorSubsystem elevator_ = ElevatorSubsystem.getInstance();

    enum State {
        UNSTOW_INTAKE, STOW_INTAKE, READY_INTAKE,
        SAFE_WRIST_INTAKE,
        SAFE_WRIST_INTAKE_FOR_ELEVATOR,
        WRIST_ELEVATOR_INTAKE_IDLE,
        MOVE_WRIST_ELEVATOR
    }

    private State state_;
    private State lastState_;

    private double wristUserGoal_ = 0;
    private double elevatorUserGoal_ = 0;
    private double intakeUserGoal_ = 0;

    private static SuperstructureSubsystem mInstance = new SuperstructureSubsystem();

    public void setElevatorWristUserGoal_(double elevator, double wrist) {
        elevatorUserGoal_ = elevator;
        wristUserGoal_ = wrist;
        state_ = State.MOVE_WRIST_ELEVATOR;
    }

    public void setIntakeUserGoal_(String mode) {
        if (mode.equals("UNSTOW")) {
            state_ = State.UNSTOW_INTAKE;
            intakeUserGoal_ = Constants.kIntakeUnstowPosition;
        } else if (mode.equals("STOW")) {
            state_ = State.STOW_INTAKE;
            intakeUserGoal_ = Constants.kIntakeStowPosition;
        } else if (mode.equals("READY")) {
            state_ = State.READY_INTAKE;
            intakeUserGoal_ = Constants.kIntakeReadyPosition;
        }
    }

    @Override
    public void fastPeriodic() {
        JointState intake_state = new JointState();
        intake_state.angle = intake_.getEncoder(); //use real current numbers, no observers
        intake_state.unprofiled_goal_angle = intake_.getIntakeController().unprofiled_goal_.position;

        JointState wrist_state = new JointState();
        wrist_state.angle = wrist_.getEncoder();
        wrist_state.unprofiled_goal_angle = wrist_.getWristController().unprofiled_goal_.position;

        double elevator_pos = 0;

        State temp_state = state_;

        //TODO: change this collision avoidance thing to a profile goal constraint system like how 971 does it
        switch (state_) {
            //Moves the wrist/elevator assembly to a safe space for the wrist to move
            case WRIST_ELEVATOR_INTAKE_IDLE: {
                intake_.setIntake_goal_pos(intakeUserGoal_);
                elevator_.setElevator_goal_pos(elevatorUserGoal_);
                wrist_.setWrist_goal_pos(wristUserGoal_);

                break;
            }
            case MOVE_WRIST_ELEVATOR: {
                if(checkIntakeSafeMoveWrist(intake_state, wrist_state, elevator_pos)) {
                    elevator_.setElevator_goal_pos(elevatorUserGoal_);
                    wrist_.setWrist_goal_pos(wristUserGoal_);
                } else {
                    state_ = State.SAFE_WRIST_INTAKE_FOR_ELEVATOR;
                }

                break;
            }
            case SAFE_WRIST_INTAKE_FOR_ELEVATOR: {
                temp_state = lastState_;
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state, elevator_pos) == false) {
                    //move the intake out of the way

                } else {
                    temp_state = State.SAFE_WRIST_INTAKE_FOR_ELEVATOR;
                    state_ = lastState_;
                }
                break;
            }
            case SAFE_WRIST_INTAKE: {
                temp_state = lastState_;
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state, elevator_pos) == false) {
                    //move the wrist/elevator out of the way in the most efficent manner so that it doesn't hit the intake

                } else {
                    //return to the 'last state' before we needed to safe the wrist
                    temp_state = State.SAFE_WRIST_INTAKE;
                    state_ = lastState_;
                }
                break;
            }
            case UNSTOW_INTAKE: {
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state, elevator_pos)) {
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
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state, elevator_pos)) {
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
                if (checkIntakeSafeMoveWrist(intake_state, wrist_state, elevator_pos)) {
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

    public boolean checkIntakeSafeMoveWrist(JointState intake, JointState wrist, double elevator_pos) {
        return true; //TODO: intakes are not always safe to move...
    }

    public static SuperstructureSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new SuperstructureSubsystem();
        }
        return mInstance;
    }

    public boolean checkBounds(double num, double low, double high) {
        if ((num >= low) && (num <= high)) {
            return true;
        } else {
            return false;
        }
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
