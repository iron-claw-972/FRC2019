package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class WristGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.00687732639197382 0.003628219473092831 ;0.0 0.44841767680572847 0.6408808531520604 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.003628219473092831 ;0.6408808531520604 ;0.0 ;");
    }

    double dt() {
        return 0.01;
    }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("1e-05 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("6.879500364742657 0.13825949712732533 0.5 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 2.0 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("0.5 ; 1.25 ; 0.0 ;");
    }
}



