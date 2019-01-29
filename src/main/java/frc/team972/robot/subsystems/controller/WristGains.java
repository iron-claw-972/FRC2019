package frc.team972.robot.subsystems.controller;

import frc.team972.robot.Constants;
import frc.team972.robot.controls.*;
import frc.team972.robot.subsystems.WristSubsystem;
import jeigen.DenseMatrix;

public class WristGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.00814271692721444 0.005981277752620018 ;0.0 0.6531004695110808 1.117170814999006 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.005981277752620018 ;1.117170814999006 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("0.0001 ;");
    }

    public static DenseMatrix K() {
        //return new DenseMatrix("0.379056641655649 0.08307340501080238 1.0 ;");
        return new DenseMatrix("0.15 0.083 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 0.8951182635404682 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("1.5 ;1.129562013499633 ;3.0441717112764493 ;");
    }
}




