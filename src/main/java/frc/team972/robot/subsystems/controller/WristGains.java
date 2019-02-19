package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class WristGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.00563308279789671 0.005073900131390999 ;0.0 0.27713562641768386 0.8398926451663994 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.005073900131390999 ;0.8398926451663994 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("1e-05 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("4.879500364742664 0.11840518733790772 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 1.1906283567965763 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("1.61397031529364 ;3.362525089129736 ;2.903947140360282 ;");
    }
}

