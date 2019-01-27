package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class ElevatorGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.004791236347425109 5.181376400930422e-05 ;0.0 0.917673229771176 0.020432962307718957 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 4.539425392599387e-07 ;0.0 0.0 -0.00025175306270045904 ;4.539425392599387e-07 -0.00025175306270045904 0.04998878885526872 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("5.181376400930422e-05 ;0.020432962307718957 ;0.0 ;");
    }

    double dt() { return 0.005; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("1e-05 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("16.41399515492959 -0.04285978261048596 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 48.94052976460639 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("0.46742789528315815 ;14.840398277521956 ;104.89633760328563 ;");
    }
}
