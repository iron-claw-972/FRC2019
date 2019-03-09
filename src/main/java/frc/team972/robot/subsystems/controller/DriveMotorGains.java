package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class DriveMotorGains {
    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.009284898191214011 0.0011729259979572845 ;0.0 0.860472855667826 0.22885554615751563 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.0011729259979572845 ;0.22885554615751563 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("1e-05 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("0.17888543819998354 0.1559745408109141 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 2.0 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("1.5772600453283918 ;1.090891131153636 ;0.0 ;");
    }
}



