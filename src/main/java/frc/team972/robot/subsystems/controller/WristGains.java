package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class WristGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.008052264544873055 0.006272574663735769 ;0.0 0.6375522968414454 1.16724284798389 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.006272574663735769 ;1.16724284798389 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("0.0001 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("0.0 0.0 0.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 0.8567197492169185 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("0.7123116042595742 ;6.102459877506496 ;2.2133137435310535 ;");
    }
}