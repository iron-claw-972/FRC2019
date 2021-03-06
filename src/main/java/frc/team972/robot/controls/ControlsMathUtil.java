package frc.team972.robot.controls;

import jeigen.DenseMatrix;

public class ControlsMathUtil {

    public static double Cap(double val, double min, double max) {
        double ret = val;
        if (val < min) {
            ret = min;
        } else if (val > max) {
            ret = max;
        }
        return ret;
    }

    public static DenseMatrix CapMatrix(DenseMatrix val, DenseMatrix min, DenseMatrix max) {
        int A = val.rows;
        int B = val.cols;
        DenseMatrix ret = new DenseMatrix(A, B);
        for (int i = 0; i < A; i++) {
            for (int j = 0; j < B; j++) {
                ret.set(i, j, Cap(val.get(i, j), min.get(i, j), max.get(i, j)));
            }
        }

        return ret;
    }

    public static DenseMatrix CloneMatrix(DenseMatrix toClone) {
        int A = toClone.rows;
        int B = toClone.cols;
        DenseMatrix ret = new DenseMatrix(A, B);
        for (int i = 0; i < A; i++) {
            for (int j = 0; j < B; j++) {
                ret.set(i, j, toClone.get(i, j));
            }
        }

        return ret;
    }


}
