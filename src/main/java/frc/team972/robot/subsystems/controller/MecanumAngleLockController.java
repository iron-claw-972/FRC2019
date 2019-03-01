package frc.team972.robot.subsystems.controller;

import frc.team972.robot.Constants;
import jeigen.DenseMatrix;

import static frc.team972.robot.Constants.kStickToVelocity;

public class MecanumAngleLockController {

    private double desired_angle = 0;
    private double last_angle = 0;

    public double update(double current_angle, double stick_rotation) {
        double current_velocity = (current_angle - last_angle) / Constants.dt;

        double desired_angular_velocity = stick_rotation * kStickToVelocity;
        desired_angle = desired_angle + (desired_angular_velocity * Constants.dt);

        double angular_p_error = desired_angle - current_angle;
        double angular_v_error = desired_angular_velocity - current_velocity;

        double u = (angular_p_error * Constants.kMecanumRotationK1) + (angular_v_error * Constants.kMecanumRotationK2);

        last_angle = current_angle;

        return u;
    }
}
