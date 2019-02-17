package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class IntakeGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.008477923610655096 0.001583903044768651 ;0.0 0.7118735883334479 0.29983008994268356 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.001583903044768651 ;0.29983008994268356 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("1e-05 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("15.8113883008419 1.116921030821255 0.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 3.3352222927030546 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("0.5011200625938521 ;1.0158102158971183 ;0.0 ;");
    }
}

