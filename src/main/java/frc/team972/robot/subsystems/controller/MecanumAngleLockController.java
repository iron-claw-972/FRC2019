package frc.team972.robot.subsystems.controller;

import frc.team972.robot.Constants;
import frc.team972.robot.controls.ControlsMathUtil;

import static frc.team972.robot.Constants.kStickToVelocity;

public class MecanumAngleLockController {

    private double desired_angle = 0;
    private double last_angle = 0;
    boolean let_go = false;

    public void reset() {
        desired_angle = 0;
        last_angle = 0;
        let_go = false;
    }

    public void setDesiredAngle(double desired_angle) {
        this.desired_angle = desired_angle;
    }

    public double update(double current_angle, double stick_rotation) {
        double current_velocity = (current_angle - last_angle) / Constants.dt;

        if(Math.abs(stick_rotation) < 0.01) {
            if(let_go == false) {
                let_go = true;
                desired_angle = current_angle;
            }
        } else {
            let_go = false;
        }

        double desired_angular_velocity = stick_rotation * kStickToVelocity;
        desired_angle = desired_angle + (desired_angular_velocity * Constants.dt);

        double angular_p_error = desired_angle - current_angle;
        double angular_v_error = desired_angular_velocity - current_velocity;

        double u = ControlsMathUtil.Cap(angular_p_error * Constants.kMecanumRotationK1, -2, 2) + (angular_v_error * Constants.kMecanumRotationK2);
        u = u * Constants.kMecanumRotationK3;

        last_angle = current_angle;

        return u;
    }
}
