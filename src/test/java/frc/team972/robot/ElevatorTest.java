package frc.team972.robot;

import frc.team972.robot.controls.StateSpacePlant;
import frc.team972.robot.subsystems.ElevatorSubsystem;
import frc.team972.robot.subsystems.controller.ElevatorController;
import jeigen.DenseMatrix;
import org.junit.Test;

public class ElevatorTest{
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(true);
    ElevatorController elevator_ = elevatorSubsystem.getElevatorController();

    StateSpacePlant plant_ = new StateSpacePlant(1, 3, 1); //Plant to simulate our elevator in this unit test

    private void Update() {
        //elevators can't fall through the floor
        if (plant_.x_.get(0, 0) < 0) {
            plant_.x_.set(0, 0, 0);
        }

        elevatorSubsystem.setHall(Math.abs(plant_.x_.get(0, 0) - Constants.kHallEffectHeight) < 2e-2);
        elevator_.Update(elevatorSubsystem);
        elevator_.SetWeights(plant_.x_.get(0, 0) > 1.0);
        DenseMatrix u_mat = new DenseMatrix(1, 1);
        u_mat.set(0, 0, elevator_.getElevator_u());

        plant_.Update(new DenseMatrix(u_mat));
    }

    private void SetGoal(double goal) {
        elevator_.SetGoal(goal);
    }

    private void SetInput(double position, boolean hall) {
        elevatorSubsystem.setEncoder(position);
        elevatorSubsystem.setHall(hall);
    }

    public void CalibrateDisabled() {
        elevatorSubsystem.setOutputs_enabled_(false);

        for (int i = 0; i < 2000; i++) {
            double h = i * .0005;
            plant_.x_.set(0, 0, h);
            elevatorSubsystem.setEncoder(plant_.y().get(0,0));
            Update();
        }
    }

    @Test
    public void testNotEnabled() {
        SetGoal(1);

        elevatorSubsystem.setOutputs_enabled_(false);

        Update();
    }

}
