package frc.team972.robot.subsystems.controller;

import frc.team972.robot.Constants;
import frc.team972.robot.controls.*;
import frc.team972.robot.subsystems.WristSubsystem;
import jeigen.DenseMatrix;


public class WristGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.00645202582674306 0.01049331603841954 ;0.0 0.38626812630144647 1.815143571819886 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.01049331603841954 ;1.815143571819886 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("0.0001 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("1.8708286933869727 0.4248823624875067 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 0.5509206078929543 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("1.4118205468248775 ;0.5729121241471241 ;0.1934494884816226 ;");
    }
}
