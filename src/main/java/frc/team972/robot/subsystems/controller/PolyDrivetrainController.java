package frc.team972.robot.subsystems.controller;

import frc.team972.robot.controls.*;
import frc.team972.robot.lib.Util;
import jeigen.DenseMatrix;

public class PolyDrivetrainController {

    public StateSpaceObserver kf_ = new StateSpaceObserver(2, 7, 3);

    private double ttrust_ = 1.1;
    int counter_ = 0;

    double throttle_ = 0.0;
    double wheel_ = 0.0;
    boolean quickturn_ = false;

    public double goal_left_velocity_ = 0.0;
    public double goal_right_velocity_ = 0.0;

    public DenseMatrix R_ = new DenseMatrix(2, 1);
    public DenseMatrix U_ = new DenseMatrix(2, 1);

    DenseMatrix A = new DenseMatrix(2, 2);
    DenseMatrix A_inv = new DenseMatrix(2, 2);
    DenseMatrix B = new DenseMatrix(2, 2);
    DenseMatrix C = new DenseMatrix(2, 2);
    DenseMatrix D = new DenseMatrix(2, 2);
    DenseMatrix U_max = new DenseMatrix(2, 1);
    DenseMatrix U_min = new DenseMatrix(2, 1);

    DenseMatrix K = new DenseMatrix(2, 2);
    DenseMatrix L = new DenseMatrix(2, 2);

    DenseMatrix x_hat = new DenseMatrix(2, 1);

    Polytope U_Poly = new Polytope(new DenseMatrix("1 0; -1 0; 0 1; 0 -1"),
            new DenseMatrix("12; 12; 12; 12"),
            new DenseMatrix("12 12 -12 -12; -12 12 12 -12"));

    public PolyDrivetrainController() {
        SetWeights();
    }

    public void SetWeights() {
        PolyDrivetrainGains.MakeVelocityDrivetrainLowLowPlantCoefficients(C, D, U_max, U_min, A, A_inv, B);
        PolyDrivetrainGains.MakeVelocityDrivetrainLowLowControllerCoefficients(K);
        PolyDrivetrainGains.MakeVelocityDrivetrainLowLowObserverCoefficients(L);
        KFDrivetrainGains.MakeKFDrivetrainLowLowPlantCoefficients(kf_.plant_.C_, kf_.plant_.D_, kf_.plant_.A_, kf_.plant_.B_);
        KFDrivetrainGains.MakeKFDrivetrainLowLowObserverCoefficients(kf_.L_);
    }

    public void SetGoal(double wheel, double throttle, boolean quickturn) {
        // Apply a sin function that's scaled to make it feel better.
        double wheel_non_linearity = 0.5;
        double quickturn_wheel_multiplier = 2;

        double angular_range = Math.PI / 2.0 * wheel_non_linearity;

        wheel_ = Math.sin(angular_range * wheel) / Math.sin(angular_range);
        wheel_ = Math.sin(angular_range * wheel_) / Math.sin(angular_range);
        wheel_ = 2.0 * wheel - wheel_;

        if (!quickturn) {
            wheel_ *= quickturn_wheel_multiplier;
        }

        final double kThrottleDeadband = 0.05;
        if (Math.abs(throttle) < kThrottleDeadband) {
            throttle_ = 0;
        } else {
            throttle_ = Math.copySign(
                    (Math.abs(throttle) - kThrottleDeadband) / (1.0 - kThrottleDeadband),
                    throttle);
        }
    }

    private double FilterVelocity(double throttle) {
        DenseMatrix FF = B.inv().mmul(DenseMatrix.eye(2).sub(A));

        DenseMatrix FF_sum = FF.sumOverRows();
        int min_FF_sum_index = 0;
        double min_FF_sum = 999999999;
        for (int i = 0; i < FF_sum.cols; i++) {
            double val = FF_sum.get(0, i);
            if (val < min_FF_sum) {
                min_FF_sum = val;
                min_FF_sum_index = i;
            }
        }
        double min_K_sum = K.col(min_FF_sum_index).sum().get(0, 0);

        double adjusted_ff_voltage =
                Util.limit(throttle * 12.0, -12.0, 12.0);

        return (adjusted_ff_voltage +
                ttrust_ * min_K_sum * (x_hat.get(0, 0) + x_hat.get(1, 0)) /
                        2.0) /
                (ttrust_ * min_K_sum + min_FF_sum);
    }

    private double MaxVelocity() {
        //TODO: implement better
        return 1;
    }

    public void Update() {
        //feed x_hat from drivetrain kf
        x_hat.set(0,0, kf_.plant_.x_.get(1, 0));
        x_hat.set(1,0, kf_.plant_.x_.get(3, 0));

        counter_++;

        DenseMatrix FF = B.inv().mmul(DenseMatrix.eye(2).sub(A));

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

        R_.set(0, 0, left_velocity);
        R_.set(1, 0, right_velocity);

        if (!quickturn_) {
            // K * R = w
            DenseMatrix equality_k = new DenseMatrix(1, 2);
            equality_k.set(0, 0, 1 + sign_svel);
            equality_k.set(0, 1, -(1 - sign_svel));
            double equality_w = 0.0;

            Polytope R_poly_hv = new Polytope(
                    U_Poly.H.mmul(K.add(FF)),
                    U_Poly.k.add(U_Poly.H.mmul(K.mmul(x_hat))),
                    (K.add(FF)).inv().mmul(Polytope.ShiftPoints(U_Poly.vertices, K.mmul(x_hat)))
            );

            R_ = CoerceGoal.DoCoerceGoal(R_poly_hv, equality_k, equality_w, R_);
        }

        DenseMatrix FF_volts = FF.mmul(R_);
        DenseMatrix U_ideal = (K.mmul(R_.sub(x_hat))).add(FF_volts);

        for (int i = 0; i < 2; i++) {
            U_.set(i, 0, Util.limit(U_ideal.get(i, 0), -12, 12));
        }

        /*
            //if we want to open_loop then just feed the observer estimate back into itself
            if (dt_config_.loop_type == LoopType::OPEN_LOOP) {
              loop_->mutable_X_hat() =
                  loop_->plant().A() * loop_->X_hat() + loop_->plant().B() * loop_->U();
            }
         */
    }
}
