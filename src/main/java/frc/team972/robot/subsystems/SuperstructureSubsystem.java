package frc.team972.robot.subsystems;

import frc.team972.robot.statemachines.SuperstructureState;
import frc.team972.robot.statemachines.SuperstructureStateMachine;

public class SuperstructureSubsystem extends Subsystem {

    WristSubsystem wrist_ = WristSubsystem.getInstance();
    ElevatorSubsystem elevator_ = ElevatorSubsystem.getInstance();

    private static SuperstructureSubsystem mInstance = new SuperstructureSubsystem();
    private SuperstructureStateMachine stateMachine = new SuperstructureStateMachine(this);

    public double elevator_des = 0;
    public double wrist_des = 0;
    public double roller_des = 0;
    public boolean hatch_des = false;

    @Override
    public void slowPeriodic() {
        stateMachine.update();

        elevator_.setElevator_goal_pos(elevator_des);
        wrist_.setWrist_goal_pos(wrist_des);
        wrist_.setRoller(roller_des);

        if(hatch_des) {
            HatchIntakeSubsystem.getInstance().setIntakeEject();
        } else {
            HatchIntakeSubsystem.getInstance().setIntakeReady();
        }

        //todo: pneumatic hatch
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
        elevator_des = 0;
        wrist_des = 0;
        roller_des = 0;
        hatch_des = false;
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