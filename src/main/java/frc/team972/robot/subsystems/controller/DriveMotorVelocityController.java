package frc.team972.robot.subsystems.controller;

import frc.team972.robot.Constants;

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
        double u_ = (kF * desired_velocity) + (velocity_error * kP);

        return u_;
    }
}
