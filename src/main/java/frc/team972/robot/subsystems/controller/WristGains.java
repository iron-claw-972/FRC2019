package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class WristGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.007817589621787227 0.007028332317008231 ;0.0 0.5978462403800927 1.2951140140094168 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.007028332317008231 ;1.2951140140094168 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("1e-05 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("1.527207096642429 0.0801727233927286 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 0.772132792312391 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("1.7398532867950036 ;11.411158821157272 ;3.6770659727687294 ;");
    }
}