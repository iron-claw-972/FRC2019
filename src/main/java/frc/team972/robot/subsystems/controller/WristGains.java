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
        return new DenseMatrix("1.914854215512682 0.749125166841897 0.05 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 0.8567197492169185 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("0.8628152389790823 ;16.39207645718683 ;5.846215076714724 ;");
    }
}

