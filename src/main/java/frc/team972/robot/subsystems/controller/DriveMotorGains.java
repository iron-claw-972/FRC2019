package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class DriveMotorGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.006722413264868324 0.0007031240358986176 ;0.0 0.42536850108185004 0.12327277699260325 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 1.9195614552572197e-05 ;0.0 0.0 -0.002647889629283179 ;1.9195614552572197e-05 -0.002647889629283179 0.09846522310415276 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.0007031240358986176 ;0.12327277699260325 ;0.0 ;");
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
        return new DenseMatrix("5.443310539518171 0.0690060362816984 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 8.112091123411645 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("1.3664965832424907 ;50.01213955122777 ;264.3961819792072 ;");
    }
}
