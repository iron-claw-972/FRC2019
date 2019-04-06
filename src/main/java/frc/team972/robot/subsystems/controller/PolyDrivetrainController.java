package frc.team972.robot.subsystems.controller;

import frc.team972.robot.controls.ControlsMathUtil;
import frc.team972.robot.controls.StateSpaceController;
import frc.team972.robot.controls.StateSpaceObserver;
import frc.team972.robot.controls.StateSpacePlant;
import frc.team972.robot.lib.Util;
import jeigen.DenseMatrix;

public class PolyDrivetrainController {
    public StateSpacePlant plant_;
    public StateSpaceController controller_;
    public StateSpaceObserver observer_;

    private double ttrust_ = 1.1;
    int counter_ = 0;

    double throttle_ = 0.0;
    double wheel_ = 0.0;
    boolean quickturn_ = false;

    double goal_left_velocity_ = 0.0;
    double goal_right_velocity_ = 0.0;

    DenseMatrix R_ = new DenseMatrix(2, 1);
    DenseMatrix U_ = new DenseMatrix(2, 1);

    public PolyDrivetrainController(StateSpacePlant plant_, StateSpaceController controller_, StateSpaceObserver observer_) {
        this.plant_ = plant_;
        this.controller_ = controller_;
        this.observer_ = observer_;

        SetWeights();
    }

    public PolyDrivetrainController() {
        plant_ = new StateSpacePlant(2, 2, 2);
        controller_ = new StateSpaceController(2, 2, 2);
        observer_ = new StateSpaceObserver(2, 2, 2);

        SetWeights();
    }

    public void SetWeights() {

    }

    private double FilterVelocity(double throttle) {
        DenseMatrix FF = (plant_.B_.t().mmul(plant_.B_)).recpr().mmul(plant_.B_.t());

        DenseMatrix FF_sum = FF.sumOverRows();
        double min_FF_sum = FF_sum.minOverCols().get(0,0);
        double min_K_sum = controller_.K_.col((int)min_FF_sum).sum().get(0,0);

        double adjusted_ff_voltage =
                Util.limit(throttle * 12.0, -12.0, 12.0);

        return (adjusted_ff_voltage +
                ttrust_ * min_K_sum * (observer_.plant_.y().get(0, 0) + observer_.plant_.y().get(1, 0)) /
                        2.0) /
                (ttrust_ * min_K_sum + min_FF_sum);
    }

    private double MaxVelocity() {
        //TODO: implement better
        return 1;
    }

    public void Update() {
        /*
          if (dt_config_.loop_type == LoopType::CLOSED_LOOP) {
            loop_->mutable_X_hat()(0, 0) = kf_->X_hat()(1, 0);
            loop_->mutable_X_hat()(1, 0) = kf_->X_hat()(3, 0);
          }
         */

        counter_++;

        DenseMatrix FF = (plant_.B_.t().mmul(plant_.B_)).recpr().mmul(plant_.B_.t());

        double fvel = FilterVelocity(throttle_);
        double sign_svel = wheel_ * ((fvel > 0.0) ? 1.0 : -1.0);
        double steering_velocity;
        if (quickturn_) {
            steering_velocity = wheel_ * MaxVelocity();
        } else {
            steering_velocity = Math.abs(fvel) * wheel_;
        }

        double left_velocity = fvel - steering_velocity;
        double right_velocity = fvel + steering_velocity;
        goal_left_velocity_ = left_velocity;
        goal_right_velocity_ = right_velocity;

        /*
            // Integrate velocity to get the position.
            // This position is used to get integral control.
         */
        R_.set(0, 0, left_velocity);
        R_.set(1, 0, right_velocity);

        if (!quickturn_) {
            // K * R = w
            DenseMatrix equality_k = new DenseMatrix(1, 2);
            equality_k.set(0, 0, 1 + sign_svel);
            equality_k.set(0, 1, -(1 - sign_svel));
            double equality_w = 0.0;

            /*
            do crazy polytope stuff in order to constraint R via constraining U
             */
        }

        DenseMatrix FF_volts = FF.mul(R_);
        DenseMatrix U_ideal = (controller_.K_.mul(R_.sub(observer_.plant_.y()))).add(FF_volts);

        for (int i = 0; i < 2; i++) {
            U_.set(0, i, Util.limit(U_.get(0, i), -12, 12));
        }

        /*
            if (dt_config_.loop_type == LoopType::OPEN_LOOP) {
              loop_->mutable_X_hat() =
                  loop_->plant().A() * loop_->X_hat() + loop_->plant().B() * loop_->U();
            }
         */

    }
}
