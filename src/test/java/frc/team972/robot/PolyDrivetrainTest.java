package frc.team972.robot;

import frc.team972.robot.controls.StateSpacePlant;
import frc.team972.robot.subsystems.controller.KFDrivetrainGains;
import frc.team972.robot.subsystems.controller.PolyDrivetrainController;
import jeigen.DenseMatrix;
import org.jfree.data.category.DefaultCategoryDataset;
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
        DefaultCategoryDataset dataset = new DefaultCategoryDataset();

        for(int t=0; t<2000; t++) {
            DenseMatrix u = new DenseMatrix("0.5; 0.5");
            if(t>1000) {
                u = new DenseMatrix("0; 0");
            }

            plant.Update(u);
            DenseMatrix sensor_input =  new DenseMatrix(3,1);
            sensor_input.set(0,0, plant.x_.get(0, 0));
            sensor_input.set(1,0, plant.x_.get(2, 0));

            polyDrivetrain.kf_.Update(u, sensor_input);

            /*
            dataset.addValue(plant.y().get(0,0), "plant_y[0]", Integer.toString(t));
            dataset.addValue(plant.y().get(1,0), "plant_y[1]", Integer.toString(t));
            dataset.addValue(plant.y().get(2,0), "plant_y[2]", Integer.toString(t));
            */

            dataset.addValue(plant.x_.get(1,0), "plant_l_velocity", Integer.toString(t));
            dataset.addValue(plant.x_.get(3,0), "plant_r_velocity", Integer.toString(t));

            dataset.addValue(polyDrivetrain.kf_.plant_.x_.get(1,0), "kf_l_velocity", Integer.toString(t));
            dataset.addValue(polyDrivetrain.kf_.plant_.x_.get(3,0), "kf_r_velocity", Integer.toString(t));


            //polyDrivetrain.kf_.Update();
            /*
            polyDrivetrain.SetGoal(0.1, 0.5, false);
            polyDrivetrain.Update();
            System.out.println("["+polyDrivetrain.U_.get(0,0) + " " + polyDrivetrain.U_.get(1,0)+"]");
            */
        }

        Graphing graphing = new Graphing("state_space", "drivetrain_plant", dataset);
        graphing.display();
        try {
            Thread.sleep(1000 * 600);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
