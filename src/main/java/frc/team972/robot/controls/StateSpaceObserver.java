package frc.team972.robot.controls;

import jeigen.DenseMatrix;

public class StateSpaceObserver {
    public StateSpacePlant plant_;
    public DenseMatrix L_;

    public StateSpaceObserver(StateSpacePlant plant, DenseMatrix L) {
        plant_ = new StateSpacePlant(ControlsMathUtil.CloneMatrix(plant.A_), ControlsMathUtil.CloneMatrix(plant.B_), ControlsMathUtil.CloneMatrix(plant.C_), ControlsMathUtil.CloneMatrix(plant.D_), ControlsMathUtil.CloneMatrix(plant.x_));
        L_ = L;
    }

    public StateSpaceObserver(int kNumInputs, int kNumStates, int kNumOutputs) {
        plant_ = new StateSpacePlant(kNumInputs, kNumStates, kNumOutputs);
        L_ = DenseMatrix.zeros(kNumStates, kNumOutputs);
    }

    // xhat_post(n) = xhat(n) + L * (y(n) - C*xhat(n))
    // xhat(n+1) = A*xhat_post(n) + B*u(n) -- done in plant.Update
    public void Update(DenseMatrix u, DenseMatrix y) {
        DenseMatrix x_add = plant_.x_.add(L_.mmul(y.sub(plant_.y())));
        plant_.x_ = x_add;
        plant_.Update(u);
    }
}
