package frc.team972.robot.controls;

public class MotionProfileConstraints {
    public double max_velocity;
    public double max_acceleration;

    public MotionProfileConstraints(double max_velocity, double max_acceleration) {
        this.max_velocity = max_velocity;
        this.max_acceleration = max_acceleration;
    }
}