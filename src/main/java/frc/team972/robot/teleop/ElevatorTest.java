package frc.team972.robot.teleop;

import frc.team972.robot.Constants;
import frc.team972.robot.controls.StateSpacePlant;
import frc.team972.robot.subsystems.ElevatorSubsystem;
import frc.team972.robot.subsystems.controller.ElevatorController;
import jeigen.DenseMatrix;

public class ElevatorTest {
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    ElevatorController elevator_ = elevatorSubsystem.getElevatorController();

    StateSpacePlant plant_ = new StateSpacePlant(1, 3, 1); //Plant to simulate our elevator in this unit test

    public void Update() {
        //elevators can't fall through the floor
        if (plant_.x_.get(0, 0) < 0) {
            plant_.x_.set(0, 0, 0);
        }

        elevatorSubsystem.setHall(Math.abs(plant_.x_.get(0, 0) - Constants.kHallEffectHeight) < 2e-2);
        elevator_.Update(elevatorSubsystem);
        elevator_.SetWeights(plant_.x_.get(0, 0) > 1.0);
        DenseMatrix u_mat = new DenseMatrix(1, 1);
        u_mat.set(1, 1, elevator_.getElevator_u());

        plant_.Update(new DenseMatrix(u_mat));
    }

}
