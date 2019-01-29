package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class WristGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.007964681129786417 0.006554632223960286 ;0.0 0.6226258868520598 1.2153125285318838 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.006554632223960286 ;1.2153125285318838 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("0.0001 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 0.8228336140071026 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("0; 0; 0");
        //return new DenseMatrix("1.232480940938629 ;23.7193568689991 ;7.337457575456882 ;");
    }
}



