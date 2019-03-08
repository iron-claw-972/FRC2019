package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class DriveMotorGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.00998332154734766 2.7356371472248876e-05 ;0.0 0.996666164973212 0.00546823085576944 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("2.7356371472248876e-05 ;0.00546823085576944 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("1e-05 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("0.28284271247461923 4.251056247815058 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 182.8745029930323 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("1.6069546155857644 ;2.9251868219507626 ;0.013254539702527842 ;");
    }
}

