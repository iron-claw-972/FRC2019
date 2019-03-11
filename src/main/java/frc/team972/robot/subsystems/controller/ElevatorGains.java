package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class ElevatorGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.006393208450102148 0.0007737466728267452 ;0.0 0.3779571460618024 0.13344369418963628 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.0007737466728267452 ;0.13344369418963628 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("1e-05 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("11.0 2.5 0.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 1.0 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("1.560830238448166 ;1.5 ;0.0 ;");
    }
}











