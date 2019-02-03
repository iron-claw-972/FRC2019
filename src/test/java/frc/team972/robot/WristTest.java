package frc.team972.robot;

import frc.team972.robot.controls.ControlsMathUtil;
import frc.team972.robot.controls.StateSpaceController;
import frc.team972.robot.controls.StateSpaceObserver;
import frc.team972.robot.controls.StateSpacePlant;
import frc.team972.robot.subsystems.WristSubsystem;
import frc.team972.robot.subsystems.controller.WristController;
import frc.team972.robot.subsystems.controller.WristGains;
import jeigen.DenseMatrix;
import org.jfree.data.category.DefaultCategoryDataset;
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

        wristSubsystem.setHall(Math.abs(plant_.x_.get(0, 0) - Constants.kWristHallEffectPosition) < 0.01); // assume our hall effect is 'precise'

        wrist_.Update(wristSubsystem);
        DenseMatrix u_mat = new DenseMatrix(1, 1);
        double u_ = wrist_.getWrist_u() * (0.925 + generateRandomNoise(0.1)); // skew input randomly by introducing output noise
        u_mat.set(0, 0, u_);

        plant_.Update(u_mat);
    }

    private double generateRandomNoise(double magnitude) {
        return (Math.random() - 0.5) * magnitude;
    }

    private void SetGoal(double goal) {
        wrist_.SetGoal(goal);
    }

    public void Calibrate(double offset) {
        wristSubsystem.setHall(false);
        wristSubsystem.setOutputs_enabled_(true);

        plant_.x_.set(0, 0, offset);

        SetGoal(0);

        for (int i = 0; i < 1000; i++) {
            wristSubsystem.setEncoder(plant_.y().get(0, 0) - offset);
            Update();
            Assert.assertEquals(wrist_.getWrist_u(), 0, 12);

        }

        Assert.assertEquals(wrist_.unprofiled_goal_.position, 0, 0.001);
        Assert.assertEquals(wrist_.profiled_goal_.position, 0, 0.001);
        Assert.assertTrue(wristSubsystem.isCalibrated());

        stayStill(offset);
    }

    public void CalibrateDisabled(double offset) {
        wristSubsystem.setHall(false);
        wristSubsystem.setOutputs_enabled_(false);

        plant_.x_.set(0, 0, offset);

        for (int i = 0; i <= 1000; i++) {
            plant_.x_.set(0, 0, offset - (((double) i / 1000.0) * offset));
            wristSubsystem.setEncoder(plant_.y().get(0, 0) - offset);
            Update();
            Assert.assertEquals(wrist_.getWrist_u(), 0, 0);
        }

        Assert.assertEquals(wrist_.unprofiled_goal_.position, 0, 0.001);
        Assert.assertEquals(wrist_.profiled_goal_.position, 0, 0.001);
        Assert.assertTrue(wristSubsystem.isCalibrated());

        stayStill(offset);
    }

    public void stayStill(double offset) {
        System.out.println("-- Staying still --");
        for (int i = 0; i <= 100; i++) {
            wristSubsystem.setEncoder(plant_.y().get(0, 0) - offset);
            Update();
            Assert.assertEquals(wrist_.getWrist_u(), 0, 0.0025);
        }

        Assert.assertEquals(wrist_.unprofiled_goal_.position, 0, 0.001);
        Assert.assertEquals(wrist_.profiled_goal_.position, 0, 0.001);
        Assert.assertTrue(wristSubsystem.isCalibrated());
        System.out.println("-- Staying still done! --");
    }

    @Test
    public void testCalibration() {
        wristSubsystem.setOutputs_enabled_(true);

        double offset = Math.toRadians(30);

        SetGoal(0); //Bottom out

        Calibrate(offset);

        Assert.assertEquals(wrist_.unprofiled_goal_.position, 0, 0.01);
        Assert.assertEquals(wrist_.profiled_goal_.position, 0, 0.01);
        Assert.assertTrue(wristSubsystem.isCalibrated());
    }

    @Test
    public void testCalibrationDisabled() {
        wristSubsystem.setOutputs_enabled_(false);

        double offset = Math.toRadians(30);

        CalibrateDisabled(offset);

        Assert.assertEquals(wrist_.unprofiled_goal_.position, 0, 0.001);
        Assert.assertEquals(wrist_.profiled_goal_.position, 0, 0.001);
        Assert.assertTrue(wristSubsystem.isCalibrated());
    }

    @Test
    public void testWristObserverRecoverIncorrectModel() {
        DefaultCategoryDataset dataset = new DefaultCategoryDataset();
        StateSpacePlant plant = plant_;

        StateSpaceController controller = wrist_.controller_;
        controller.K_ = new DenseMatrix("0.0 0.0");
        controller.A_ = ControlsMathUtil.CloneMatrix(plant.A_);
        controller.Kff_ = (plant.B_.t().mmul(plant.B_)).recpr().mmul(plant.B_.t());
        controller.r_ = new DenseMatrix("0.0; 0.0");

        StateSpaceObserver observer = wrist_.observer_;

        // change model up from observer model
        plant.A_.set(0, 1, plant.A_.get(0, 1) * 1.23);
        plant.A_.set(1, 1, plant.A_.get(1, 1) * 1.1);
        plant.A_.set(0, 2, plant.A_.get(1, 2) * 0.15);

        observer.plant_.x_ = new DenseMatrix("0.0; 0; 0"); //initial plant "error" condition

        for (int t = 0; t < 1000; t++) {
            DenseMatrix u = new DenseMatrix("1.0");
            if (t > 500) {
                u.set(0, 0, 0.1);
            }
            DenseMatrix noised_sensor = plant.y().add(generateRandomNoise(Math.toRadians(1))); // randomly -+ 1 deg per sensor sample
            noised_sensor = noised_sensor.mul(1.0);
            observer.Update(u.mul(1.0), noised_sensor);
            plant.Update(u);

            dataset.addValue(plant.x_.get(0, 0), "plant_x[1]", Integer.toString(t));
            dataset.addValue(observer.plant_.x_.get(0, 0), "obs_x[1]", Integer.toString(t));
            dataset.addValue(plant.x_.get(1, 0), "plant_x[2]", Integer.toString(t));
            dataset.addValue(observer.plant_.x_.get(1, 0), "obs_x[2]", Integer.toString(t));

            dataset.addValue(noised_sensor.get(0, 0), "sensor[1]", Integer.toString(t));

        }

        Assert.assertEquals(plant.x_.get(0, 0), observer.plant_.x_.get(0, 0), 0.01);
        Assert.assertEquals(plant.x_.get(1, 0), observer.plant_.x_.get(1, 0), 0.1);

        /*
        Graphing graphing = new Graphing("state_space", "wrist_observer", dataset);
        graphing.display();
        try {
            Thread.sleep(1000 * 600);
        } catch (Exception e) {
            e.printStackTrace();
        }
        */
    }

    @Test
    public void testWristMove() {
        // make the plant model less accurate
        plant_.B_.set(0, 0, plant_.B_.get(0, 0) * 1.6);
        plant_.B_.set(1, 0, plant_.B_.get(1, 0) * 1.5);

        double offset = Math.toRadians(30);
        DefaultCategoryDataset dataset = new DefaultCategoryDataset();

        CalibrateDisabled(offset);

        Assert.assertTrue(wristSubsystem.isCalibrated());
        wristSubsystem.setOutputs_enabled_(true);

        wrist_.SetGoal(Math.toRadians(45));
        wrist_.SetWeights();

        for (int i = 0; i < 1000; i++) {
            dataset.addValue(plant_.y().get(0, 0), "plant_y", Integer.toString(i));
            dataset.addValue(plant_.x_.get(1, 0), "plant_x[1]", Integer.toString(i));

            dataset.addValue(wrist_.profiled_goal_.position, "profiled_pos", Integer.toString(i));
            dataset.addValue(wrist_.profiled_goal_.velocity, "profiled_velocity", Integer.toString(i));

            dataset.addValue(wrist_.observer_.plant_.y().get(0, 0), "observer_y", Integer.toString(i));
            dataset.addValue(wrist_.observer_.plant_.x_.get(1, 0), "observer_x[1]", Integer.toString(i));
            dataset.addValue(wrist_.getWrist_u(), "u", Integer.toString(i));

            wristSubsystem.setEncoder(plant_.y().get(0, 0) - offset + generateRandomNoise(Math.toRadians(0.05)));
            Update();
            Assert.assertEquals(wrist_.getWrist_u(), 0, 12);
        }

        Assert.assertEquals(plant_.y().get(0, 0), wrist_.unprofiled_goal_.position, 0.01);
        Assert.assertEquals(wrist_.observer_.plant_.y().get(0, 0), wrist_.unprofiled_goal_.position, 0.01);
        Assert.assertEquals(wrist_.profiled_goal_.position, wrist_.unprofiled_goal_.position, 0.01);

        Graphing graphing = new Graphing("state_space", "wristMoveAngle", dataset);
        graphing.display();
        try {
            Thread.sleep(1000 * 600);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


}