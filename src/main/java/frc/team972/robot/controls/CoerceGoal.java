package frc.team972.robot.controls;

import jeigen.DenseMatrix;

public class CoerceGoal {

    /*
        co·erce
        /kōˈərs/
        verb
        persuade (an unwilling person) to do something by using force or threats.
        "they were coerced into silence"
     */

    public static DenseMatrix DoCoerceGoal(Polytope region, DenseMatrix K, double w, DenseMatrix R) {
        if (region.IsInside(R)) {
            return R;
        }
        DenseMatrix perpendicular_vector = K.t().normalized();
        DenseMatrix parallel_vector  = new DenseMatrix(2, 1);
        parallel_vector.set(0, 0, perpendicular_vector.get(1, 0));
        parallel_vector.set(1, 0, -perpendicular_vector.get(0, 0));

        DenseMatrix projectedh = region.H.mmul(parallel_vector);
        DenseMatrix projectedk = region.k.sub(region.H.mmul(perpendicular_vector.mul(w)));

        double min_boundary = -99999999;
        double max_boundary = 99999999;
        for (int i = 0; i < 4; i++) {
            if (projectedh.get(i, 0) > 0) {
                max_boundary = Math.min(max_boundary, projectedk.get(i, 0) / projectedh.get(i, 0));
            } else {
                min_boundary = Math.max(min_boundary, projectedk.get(i, 0) / projectedh.get(i, 0));
            }
        }

        DenseMatrix vertices = new DenseMatrix(1, 2);
        vertices.set(0, 0, max_boundary);
        vertices.set(0, 1, min_boundary);

        if (max_boundary > min_boundary) {
            double min_distance_sqr = 0;
            DenseMatrix closest_point = null;
            for (int i = 0; i < vertices.cols; i++) {
                DenseMatrix point;
                point = (parallel_vector.mul(vertices.get(0, i))).add(perpendicular_vector.mul(w));
                double length = (R.sub(point)).squaredNorm();
                if ((i==0) || (length < min_distance_sqr)) {
                    closest_point = point;
                    min_distance_sqr = length;
                }
            }
            return closest_point;
        } else {
            DenseMatrix region_vertices = region.vertices;
            double min_distance = 99999999;
            int closest_i = 0;
            for (int i = 0; i < region_vertices.rows; i++) {
                double length = Math.abs(perpendicular_vector.t().mmul(region_vertices.col(i)).get(0, 0));
                if ((i == 0) || (length < min_distance)) {
                    closest_i = i;
                    min_distance = length;
                }
            }
            DenseMatrix ans = new DenseMatrix(2, 1);
            ans.set(0, 0, region_vertices.get(0, closest_i));
            ans.set(1, 0, region_vertices.get(1, closest_i));
            return ans;
        }

    }
}
