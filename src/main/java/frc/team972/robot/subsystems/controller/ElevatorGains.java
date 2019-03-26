package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

import jeigen.DenseMatrix;

import jeigen.DenseMatrix;

public class ElevatorGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.00756484819598747 0.0007008927252543203 ;0.0 0.5561296890612022 0.12775608953033304 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 2.4608909062816955e-06 ;0.0 0.0 -0.00043248251127062467 ;2.4608909062816955e-06 -0.00043248251127062467 0.014801292056027713 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.0007008927252543203 ;0.12775608953033304 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("0.0001 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("12.1666666666667345 2.0 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 1 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("0.5 ;2.0290459426272194 ; 0.0 ;");
    }
}















