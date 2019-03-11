package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

import jeigen.DenseMatrix;

public class ElevatorGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.008586527925338995 0.00040682978894687906 ;0.0 0.7312971983847448 0.07733884951125544 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 2.032950102467479e-06 ;0.0 0.0 -0.00045818092672426865 ;2.032950102467479e-06 -0.00045818092672426865 0.02490308746973273 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.00040682978894687906 ;0.07733884951125544 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("0.001 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("9.000000000000009 1.7246430136911568 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 12.930112179318957 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("0.2505513604929526 ;0.010886165465222256 ;0.025664825456869576 ;");
    }
}












