package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

import jeigen.DenseMatrix;

import jeigen.DenseMatrix;

public class ElevatorGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.006936444461149615 0.00088176178054063 ;0.0 0.4573353524946653 0.1561913729823607 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 5.773257551509884e-06 ;0.0 0.0 -0.0008501498907233547 ;5.773257551509884e-06 -0.0008501498907233547 0.024419288254708838 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.00088176178054063 ;0.1561913729823607 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("0.001 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("9.000000000000018 1.608017811015691 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 6.402402264002981 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("0.25086821528185516 ;0.01724394190780159 ;0.058917131499979895 ;");
    }
}












