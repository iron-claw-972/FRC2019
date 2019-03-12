package frc.team972.robot.subsystems;

import frc.team972.robot.statemachines.SuperstructureState;
import frc.team972.robot.statemachines.SuperstructureStateMachine;

public class SuperstructureSubsystem extends Subsystem {

    WristSubsystem wrist_ = WristSubsystem.getInstance();
    ElevatorSubsystem elevator_ = ElevatorSubsystem.getInstance();
    private static SuperstructureSubsystem mInstance = new SuperstructureSubsystem();

    private SuperstructureStateMachine stateMachine = new SuperstructureStateMachine();

    @Override
    public void fastPeriodic() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }

    public SuperstructureState getState() {
        return stateMachine.currentState;
    }

    public void setState(SuperstructureState state) {
        stateMachine.currentState = state;
    }

    public static SuperstructureSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new SuperstructureSubsystem();
        }
        return mInstance;
    }

}