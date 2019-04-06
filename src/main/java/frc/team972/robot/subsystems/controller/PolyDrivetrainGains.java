package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class PolyDrivetrainGains {

    public void MakeVelocityDrivetrainLowLowPlantCoefficients(DenseMatrix C, DenseMatrix D, DenseMatrix U_max, DenseMatrix U_min, DenseMatrix A, DenseMatrix A_inv, DenseMatrix B) {
        C.set(0, 0, 1.0);
        C.set(0, 1, 0.0);
        C.set(1, 0, 0.0);
        C.set(1, 1, 1.0);
        D.set(0, 0, 0.0);
        D.set(0, 1, 0.0);
        D.set(1, 0, 0.0);
        D.set(1, 1, 0.0);
        U_max.set(0, 0, 12.0);
        U_max.set(1, 0, 12.0);
        U_min.set(0, 0, -12.0);
        U_min.set(1, 0, -12.0);
        A.set(0, 0, 0.9792639626306658);
        A.set(0, 1, 0.01004709387831373);
        A.set(1, 0, 0.010047093878313733);
        A.set(1, 1, 0.9792639626306661);
        A_inv.set(0, 0, 1.0212826302319988);
        A_inv.set(0, 1, -0.010478198783775745);
        A_inv.set(1, 0, -0.010478198783775747);
        A_inv.set(1, 1, 1.0212826302319986);
        B.set(0, 0, 0.009359151946675777);
        B.set(0, 1, -0.004534727467684608);
        B.set(1, 0, -0.0045347274676846095);
        B.set(1, 1, 0.009359151946675779);
    }
}