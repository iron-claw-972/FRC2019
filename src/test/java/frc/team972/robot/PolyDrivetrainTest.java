package frc.team972.robot;

import frc.team972.robot.controls.StateSpacePlant;
import frc.team972.robot.subsystems.controller.KFDrivetrainGains;
import frc.team972.robot.subsystems.controller.PolyDrivetrainController;
import jeigen.DenseMatrix;
import org.junit.Test;

import static org.junit.Assert.assertTrue;

public class PolyDrivetrainTest {

    static StateSpacePlant plant = new StateSpacePlant(2, 7, 3);
    static {
        KFDrivetrainGains.MakeKFDrivetrainLowLowPlantCoefficients(plant.C_, plant.D_, plant.A_, plant.B_);
    }
    PolyDrivetrainController polyDrivetrain = new PolyDrivetrainController();

    @Test
    public void polytrainTest() {
        for(int i=0; i<5; i++) {
            plant.Update(new DenseMatrix("0; 0"));
            System.out.println(plant.y().get(0,0) + " " + plant.y().get(1,0) + " " + plant.y().get(2,0));
            //polyDrivetrain.kf_.Update();
            /*
            polyDrivetrain.SetGoal(0.1, 0.5, false);
            polyDrivetrain.Update();
            System.out.println("["+polyDrivetrain.U_.get(0,0) + " " + polyDrivetrain.U_.get(1,0)+"]");
            */
        }
    }

}
