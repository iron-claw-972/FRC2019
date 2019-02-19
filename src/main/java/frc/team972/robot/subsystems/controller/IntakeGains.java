package frc.team972.robot.subsystems.controller;

import jeigen.DenseMatrix;

public class IntakeGains {

    public static DenseMatrix A() {
        return new DenseMatrix("1.0 0.008755843126226166 0.0012946944544540066 ;0.0 0.7619426853146791 0.24772718912078645 ;0.0 0.0 1.0 ;");
    }

    public static DenseMatrix Q() {
        return new DenseMatrix("0.0 0.0 0.0 ;0.0 0.0 0.0 ;0.0 0.0 0.0 ;");
    }

    public static DenseMatrix C() {
        return new DenseMatrix("1.0 0.0 0.0 ;");
    }

    public static DenseMatrix B() {
        return new DenseMatrix("0.0012946944544540066 ;0.24772718912078645 ;0.0 ;");
    }

    double dt() { return 0.01; }

    public static DenseMatrix D() {
        return new DenseMatrix("0.0 ;");
    }

    public static DenseMatrix R() {
        return new DenseMatrix("1e-05 ;");
    }

    public static DenseMatrix K() {
        return new DenseMatrix("35.622776601683775 5.401762222574543 1.0 ;");
    }

    public static DenseMatrix Kff() {
        return new DenseMatrix("0.0 4.036698610068277 0.0 ;");
    }

    public static DenseMatrix L() {
        return new DenseMatrix("0.4981315144359132 ;0.9146928601437839 ;0.0 ;");
    }
}






