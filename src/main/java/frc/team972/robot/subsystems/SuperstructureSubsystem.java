package frc.team972.robot.subsystems;

import static frc.team972.robot.Constants.*;

public class SuperstructureSubsystem extends Subsystem {

    IntakeSubsystem intake_ = IntakeSubsystem.getInstance();
    WristSubsystem wrist_ = WristSubsystem.getInstance();
    ElevatorSubsystem elevator_ = ElevatorSubsystem.getInstance();

    private double wristUserGoal_ = 0;
    private double elevatorUserGoal_ = 0;
    private double intakeUserGoal_ = 0;

    private double kSafetyMargin = 2.0; //degrees

    private static SuperstructureSubsystem mInstance = new SuperstructureSubsystem();

    public void setElevatorWristUserGoal_(double elevator, double wrist) {
        elevatorUserGoal_ = elevator;
        wristUserGoal_ = wrist;
    }

    public void setIntakeUserGoal_(double intake) {
        intakeUserGoal_ = intake;
    }

    @Override
    public void fastPeriodic() {
        JointState intake_state = new JointState();
        intake_state.angle = intake_.getEncoder(); //use real current numbers, no observers
        intake_state.unprofiled_goal_angle = intake_.getIntakeController().unprofiled_goal_.position;

        JointState wrist_state = new JointState();
        wrist_state.angle = wrist_.getEncoder();
        wrist_state.unprofiled_goal_angle = wrist_.getWristController().unprofiled_goal_.position;

        PositionState elevator_state = new PositionState(); //angle is linear lol
        elevator_state.linear = elevator_.getEncoder();
        elevator_state.unprofiled_goal_linear = elevator_.getElevatorController().unprofiled_goal_.position;

        double wrist_goal = wristUserGoal_;
        double elevator_goal = elevatorUserGoal_;
        double intake_goal = intakeUserGoal_;


        // ---- COLLISION AVOIDANCE

        //if we move our wrist more than 90 degrees, and the elevator is too far down, we will hit stuff
        if((wrist_state.angle > kWristIntakeMaxAngleBeforeProtrudeDown + kSafetyMargin) || (wrist_goal > 90.0 + kSafetyMargin)) {
            if(elevator_state.linear < 0.2) {
                wrist_goal = 90;
            }
        }

        if((elevator_state.linear < kElevatorMinPosBeforeIntakeCollision) && (elevator_goal < kElevatorMinPosBeforeIntakeCollision)) {
            boolean intake_in_safe_side_for_elevator_max_down = false;
            if(intake_state.angle >= kIntakePositionRangeBeforeIntakeCollision - kSafetyMargin) {

            }
        }


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

class PositionState {
    double linear;
    double linear_velocity;
    double goal_linear;
    double goal_linear_velocity;
    double unprofiled_goal_linear;
    double unprofiled_goal_linear_velocity;
}
