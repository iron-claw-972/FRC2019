package frc.team972.robot;

import frc.team972.robot.controls.StateSpacePlant;
import frc.team972.robot.subsystems.ElevatorSubsystem;
import frc.team972.robot.subsystems.controller.ElevatorController;
import frc.team972.robot.subsystems.controller.ElevatorGains;
import jeigen.DenseMatrix;
import org.jfree.data.category.DefaultCategoryDataset;
import org.junit.Assert;
import org.junit.Test;


public class ElevatorTest {
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(true);
    ElevatorController elevator_ = elevatorSubsystem.getElevatorController();

    StateSpacePlant plant_ = new StateSpacePlant(ElevatorGains.A(), ElevatorGains.B(), ElevatorGains.C(), ElevatorGains.D()); //Plant to simulate our elevator in this unit test

    private void Update() {
        // elevators can't fall through the floor
        if (plant_.x_.get(0, 0) < 0) {
            plant_.x_.set(0, 0, 0);
        }
        elevatorSubsystem.setHall(Math.abs(plant_.x_.get(0, 0) - Constants.kElevatorHallEffectPosition) < 0.01); // assume our hall effect is 'precise'
        elevator_.Update(elevatorSubsystem);
        DenseMatrix u_mat = new DenseMatrix(1, 1);
        u_mat.set(0, 0, elevator_.getElevator_u());

        plant_.Update(new DenseMatrix(u_mat));
    }

    private void SetGoal(double goal) {
        elevator_.SetGoal(goal);
    }

    public void Calibrate(double offset) {
        elevatorSubsystem.setHall(false);
        elevatorSubsystem.setOutputs_enabled_(true);

        plant_.x_.set(0,0, offset);

        SetGoal(0); //Bottom out

        for (int i = 0; i < 1000; i++) {
            elevatorSubsystem.setEncoder(plant_.y().get(0, 0) - offset);
            Update();
            Assert.assertEquals(elevator_.getElevator_u(), 0, 12);
        }

        Assert.assertEquals(elevator_.unprofiled_goal_.position, 0, 0.001);
        Assert.assertEquals(elevator_.profiled_goal_.position, 0, 0.001);
        Assert.assertTrue(elevatorSubsystem.isCalibrated());
    }

    public void CalibrateDisabled(double offset) {
        elevatorSubsystem.setHall(false);
        elevatorSubsystem.setOutputs_enabled_(false);

        plant_.x_.set(0,0, offset);

        for (int i = 0; i < 1000; i++) {
            plant_.x_.set(0,0, offset - ((double)i/1000.0) * offset);
            elevatorSubsystem.setEncoder(plant_.y().get(0, 0) - offset);
            Update();
            Assert.assertEquals(elevator_.getElevator_u(), 0, 12);
        }

        Assert.assertEquals(elevator_.unprofiled_goal_.position, 0, 0.001);
        Assert.assertEquals(elevator_.profiled_goal_.position, 0, 0.001);
        Assert.assertTrue(elevatorSubsystem.isCalibrated());
    }

    @Test
    public void testNotEnabled() {
        SetGoal(1);

        elevatorSubsystem.setOutputs_enabled_(false);

        Update();
    }

    private double generateRandomNoise(double magnitude) {
        return (Math.random() - 0.5) * magnitude;
    }

    @Test
    public void testCalibration() {
        elevatorSubsystem.setOutputs_enabled_(true);

        double offset = 0.5;

        SetGoal(0); //Bottom out

        Calibrate(offset);

        Assert.assertEquals(elevator_.unprofiled_goal_.position, 0, 0.001);
        Assert.assertEquals(elevator_.profiled_goal_.position, 0, 0.001);
        Assert.assertTrue(elevatorSubsystem.isCalibrated());
    }

    @Test
    public void testCalibrationDisabled() {
        elevatorSubsystem.setOutputs_enabled_(false);

        double offset = 0.5;

        CalibrateDisabled(offset);

        Assert.assertEquals(elevator_.unprofiled_goal_.position, 0, 0.001);
        Assert.assertEquals(elevator_.profiled_goal_.position, 0, 0.001);
        Assert.assertTrue(elevatorSubsystem.isCalibrated());
    }

    @Test
    public void testElevatorMoveHeight() {
        double offset = 0.2;
        DefaultCategoryDataset dataset = new DefaultCategoryDataset();

        Calibrate(offset);
        Assert.assertTrue(elevatorSubsystem.isCalibrated());
        elevatorSubsystem.setOutputs_enabled_(true);

        elevator_.SetGoal(0.6);
        elevator_.SetWeights(false);

        for (int i = 0; i < 1000; i++) {
            dataset.addValue(plant_.y().get(0, 0), "plant_y", Integer.toString(i));
            dataset.addValue(plant_.x_.get(1, 0), "plant_x[1]", Integer.toString(i));
            dataset.addValue(elevator_.profiled_goal_.position, "profiled_pos", Integer.toString(i));
            dataset.addValue(elevator_.observer_.plant_.y().get(0, 0), "observer_y", Integer.toString(i));
            dataset.addValue(elevator_.observer_.plant_.x_.get(1, 0), "observer_x[1]", Integer.toString(i));
            dataset.addValue(elevator_.getElevator_u() * (1.0 / 12.0), "u", Integer.toString(i));
            //dataset.addValue((elevatorSubsystem.getEncoder() - last_pos) * 1000, "d from pid", Integer.toString(i));

            elevatorSubsystem.setEncoder(plant_.y().get(0, 0) - offset + generateRandomNoise(0.0005));

            Update();
            Assert.assertEquals(elevator_.getElevator_u(), 0, 12);
        }

        Assert.assertEquals(plant_.y().get(0,0), 0.6, 0.01);
        Assert.assertEquals(elevator_.observer_.plant_.y().get(0, 0), 0.6, 0.01);
        Assert.assertEquals(elevator_.unprofiled_goal_.position, 0.6, 0.01);
        Assert.assertEquals(elevator_.profiled_goal_.position, 0.6, 0.01);

        /*
        Graphing graphing = new Graphing("state_space", "elevatorMoveHeight", dataset);
        graphing.display();
        try {
            Thread.sleep(1000 * 600);
        } catch (Exception e) {
            e.printStackTrace();
        }
        */
    }

}
