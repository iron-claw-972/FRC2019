package frc.team972.robot;

import frc.team972.robot.controls.ControlsMathUtil;
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
    public void drivetrainTestObserver() {
        DefaultCategoryDataset dataset = new DefaultCategoryDataset();

        for (int t = 0; t < 2000; t++) {
            DenseMatrix u = new DenseMatrix("0.5; 0.3");
            if (t > 1000) {
                u = new DenseMatrix("0; 0");
            }

            plant.Update(u);
            DenseMatrix sensor_input = new DenseMatrix(3, 1);
            sensor_input.set(0, 0, plant.x_.get(0, 0)); //left drivetrain velocity
            sensor_input.set(1, 0, plant.x_.get(2, 0)); //right drivetrain velocity
            sensor_input.set(2, 0, plant.y().get(2, 0)); //angular velocity

            polyDrivetrain.kf_.Update(u, sensor_input);

            dataset.addValue(plant.x_.get(1, 0), "plant_l_velocity", Integer.toString(t));
            dataset.addValue(plant.x_.get(3, 0), "plant_r_velocity", Integer.toString(t));

            dataset.addValue(polyDrivetrain.kf_.plant_.x_.get(1, 0), "kf_l_velocity", Integer.toString(t));
            dataset.addValue(polyDrivetrain.kf_.plant_.x_.get(3, 0), "kf_r_velocity", Integer.toString(t));
        }

        Graphing graphing = new Graphing("state_space", "drivetrain_plant", dataset);
        graphing.display();
        try {
            Thread.sleep(1000 * 600);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Test
    public void polytrainTest() {
        DefaultCategoryDataset dataset = new DefaultCategoryDataset();
        DenseMatrix u = new DenseMatrix("0.0; 0.0");

        for (int t = 0; t < 2000; t++) {
            if (t > 1500) {
                polyDrivetrain.SetGoal(0.1, 0.8, false);
            } else if (t > 1000) {
                polyDrivetrain.SetGoal(0.1, 0.5, false);
            } else {
                polyDrivetrain.SetGoal(0.0, 0.5, false);
            }


            plant.Update(u);
            DenseMatrix sensor_input = new DenseMatrix(3, 1);
            sensor_input.set(0, 0, plant.x_.get(0, 0)); //left drivetrain velocity
            sensor_input.set(1, 0, plant.x_.get(2, 0)); //right drivetrain velocity
            sensor_input.set(2, 0, plant.y().get(2, 0)); //angular velocity

            polyDrivetrain.kf_.Update(u, sensor_input);
            polyDrivetrain.Update();
            System.out.println("[" + polyDrivetrain.U_.get(0, 0) + " " + polyDrivetrain.U_.get(1, 0) + "]");
            u = ControlsMathUtil.CloneMatrix(polyDrivetrain.U_);

            dataset.addValue(plant.x_.get(1, 0), "plant_l_velocity", Integer.toString(t));
            dataset.addValue(plant.x_.get(3, 0), "plant_r_velocity", Integer.toString(t));
            dataset.addValue(plant.y().get(2, 0), "angular_velocity", Integer.toString(t));

            dataset.addValue(polyDrivetrain.U_.get(0, 0), "u_left", Integer.toString(t));
            dataset.addValue(polyDrivetrain.U_.get(1, 0), "u_right", Integer.toString(t));

            dataset.addValue(polyDrivetrain.goal_left_velocity_, "l_velocity_goal", Integer.toString(t));
            dataset.addValue(polyDrivetrain.goal_right_velocity_, "r_velocity_goal", Integer.toString(t));
        }

        Graphing graphing = new Graphing("state_space", "polydrivetrain", dataset);
        graphing.display();
        try {
            Thread.sleep(1000 * 600);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
