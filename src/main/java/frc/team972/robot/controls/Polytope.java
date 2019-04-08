package frc.team972.robot.controls;

import jeigen.DenseMatrix;

public class Polytope {

    public DenseMatrix H;
    public DenseMatrix k;
    public DenseMatrix vertices;

    public Polytope(int num_of_dimensions, int num_constraints, int num_vertices) {
        H = new DenseMatrix(num_constraints, num_of_dimensions);
        k = new DenseMatrix(num_constraints, 1);
        vertices = new DenseMatrix(num_of_dimensions, num_vertices);
    }

    public Polytope(int num_of_dimensions, int num_constraints) {
        H = new DenseMatrix(num_constraints, num_of_dimensions);
        k = new DenseMatrix(num_constraints, 1);
    }

    //HV polytope
    public Polytope(DenseMatrix H_, DenseMatrix k_, DenseMatrix vertices_) {
        H = H_;
        k = k_;
        vertices = vertices_;
    }

    public boolean IsInside(DenseMatrix point) {
        for (int i = 0; i < k.rows; ++i) {
            double ev = (H.row(i).mmul(point).get(0, 0));
            if (ev > k.get(i, 0)) {
                return false;
            }
        }
        return true;
    }

    public static DenseMatrix ShiftPoints(DenseMatrix vertices, DenseMatrix offset) {
        DenseMatrix ans = ControlsMathUtil.CloneMatrix(vertices);
        for (int i = 0; i < vertices.cols; i++) {
            for (int z = 0; z < ans.rows; z++) {
                ans.set(z, i, ans.get(z, i) + offset.get(z, 0));
            }
        }
        return ans;
    }

}
