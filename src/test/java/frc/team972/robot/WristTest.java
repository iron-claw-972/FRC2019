package frc.team972.robot;

import frc.team972.robot.controls.StateSpacePlant;
import frc.team972.robot.subsystems.WristSubsystem;
import frc.team972.robot.subsystems.controller.WristController;
import frc.team972.robot.subsystems.controller.WristGains;
import jeigen.DenseMatrix;
import org.junit.Assert;
import org.junit.Test;

public class WristTest {
    WristSubsystem wristSubsystem = new WristSubsystem(true);
    WristController wrist_ = wristSubsystem.getWristController();

    StateSpacePlant plant_ = new StateSpacePlant(WristGains.A(), WristGains.B(), WristGains.C(), WristGains.D());

    private void Update() {
        if (plant_.x_.get(0, 0) < 0) {
            plant_.x_.set(0, 0, 0);
        }

        wristSubsystem.setHall(Math.abs(plant_.x_.get(0, 0)) < 0.001); // assume our hall effect is 'precise'

        wrist_.Update(wristSubsystem);
        DenseMatrix u_mat = new DenseMatrix(1, 1);
        u_mat.set(0, 0, wrist_.getWrist_u());

        plant_.Update(new DenseMatrix(u_mat));
    }

    private void SetGoal(double goal) {
        wrist_.SetGoal(goal);
    }

    @Test
    public void testCalibration() {
        wristSubsystem.setEncoder(0);
        wristSubsystem.setHall(false);
        wristSubsystem.setOutputs_enabled_(true);

        double offset = Math.toRadians(30);
        plant_.x_.set(0,0, offset);

        SetGoal(0); //Bottom out

        for (int i = 0; i < 1000; i++) {
            wristSubsystem.setEncoder(plant_.y().get(0, 0) - offset);
            Update();
            Assert.assertEquals(wrist_.getWrist_u(), 0, 12);

            System.out.println(plant_.x_.get(0,0) + " " + wrist_.getWrist_u());
        }

        Assert.assertEquals(wrist_.unprofiled_goal_.position, 0, 0.001);
        Assert.assertEquals(wrist_.profiled_goal_.position, 0, 0.001);
        Assert.assertTrue(wristSubsystem.isCalibrated());
    }


}
