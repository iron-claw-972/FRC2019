package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class PolyDrivetrainGains {

    public static void MakeVelocityDrivetrainLowLowPlantCoefficients(DenseMatrix C, DenseMatrix D, DenseMatrix U_max, DenseMatrix U_min, DenseMatrix A, DenseMatrix A_inv, DenseMatrix B) {
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
        A.set(0, 0, 0.9787326675545024);
        A.set(0, 1, 0.009515798802150225);
        A.set(1, 0, 0.009515798802150225);
        A.set(1, 1, 0.9787326675545024);
        A_inv.set(0, 0, 1.0218260516848774);
        A_inv.set(0, 1, -0.009934777330897033);
        A_inv.set(1, 0, -0.009934777330897033);
        A_inv.set(1, 1, 1.0218260516848776);
        B.set(0, 0, 0.009598950479913788);
        B.set(0, 1, -0.004294928934446597);
        B.set(1, 0, -0.004294928934446598);
        B.set(1, 1, 0.009598950479913788);

    }

    public static void MakeVelocityDrivetrainLowLowControllerCoefficients(DenseMatrix K) {
        K.set(0, 0, 10.809926940701153);
        K.set(0, 1, 5.828102448917297);
        K.set(1, 0, 5.828102448917297);
        K.set(1, 1, 10.809926940701155);
    }

    public static void MakeVelocityDrivetrainLowLowObserverCoefficients(DenseMatrix L) {

        L.set(0, 0, 0.9587326675545024);
        L.set(0, 1, 0.009515798802150225);
        L.set(1, 0, 0.009515798802150225);
        L.set(1, 1, 0.9587326675545024);
    }

}