package frc.team972.robot.subsystems.controller;

import frc.team972.robot.Constants;
import frc.team972.robot.controls.*;
import frc.team972.robot.subsystems.WristSubsystem;
import jeigen.DenseMatrix;

public class WristGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.0013897357939161785 0.0037371397046132774 ;0.0 0.030567077765148467 1.0035016989997612 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.0037371397046132774 ;1.0035016989997612 ;0.0 ;");
    }

    double dt() { return 0.005; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("0.01 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("3.1622776601683515 0.4275106562032513 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 0.9965105201084846 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("0.07206980317445527 ;0.22676446930829053 ;0.21907784138539232 ;");
    }
}