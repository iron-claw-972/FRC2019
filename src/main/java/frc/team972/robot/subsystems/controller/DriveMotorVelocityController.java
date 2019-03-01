package frc.team972.robot.subsystems.controller;

public class DriveMotorVelocityController {

    private double kP;
    private double kF;
    private double u;

    public DriveMotorVelocityController(double kP, double kF, double u) {
        this.kP = kP;
        this.kF = kF;
        this.u = u;
    }

    public double update(double current_velocity, double desired_velocity) {
        double velocity_error = (desired_velocity - current_velocity);
        u = u + velocity_error * kP;
        double u_ = u + (kF * desired_velocity);

        return u_;
    }
}
