package frc.team972.robot.statemachines;

import frc.team972.robot.Constants;
import frc.team972.robot.subsystems.ElevatorSubsystem;
import frc.team972.robot.subsystems.SuperstructureSubsystem;
import frc.team972.robot.subsystems.WristSubsystem;
import frc.team972.robot.teleop.ControlBoard;

public class SuperstructureStateMachine {

    public static final double kWristStartOffset = 22.0;
    public static final double kWristReadyRaiseAngle = 90 + 25.0;
    public static final double kWristReadyOuttakeBallAngle = 90 + 25.0;
    public static final double kWristReadyOuttakeFlatAngle = 150;
    public static final double kWristReadyFlatAngle = 179.0;


    public static final double kElevatorReadyFlat = 0.0;
    public static final double kElevatorReadyLip = 0.0;
    public static final double kElevatorLevelOne = 0.0;
    public static final double kElevatorLevelTwo = 0.0;
    public static final double kElevatorLevelThree = 0.0;

    public static final double kElevatorHatch = 0.0;
    public static final double kElevatorLevelOneH = 0.0;
    public static final double kElevatorLevelTwoH = 0.0;
    public static final double kElevatorLevelThreeH = 0.0;


    public static final double kRollerIntakePower = 0.5;
    public static final double kRollerOuttakePower = -0.5;

    public SuperstructureState currentState = SuperstructureState.READY_WRIST_RAISE;
    SuperstructureSubsystem superstructureSubsystem = SuperstructureSubsystem.getInstance();

    public void update() {
        switch (currentState) {
            // - - - Ready States
            case READY_WRIST_RAISE: {
                setWrist(kWristReadyRaiseAngle);
                setElevator(kElevatorReadyFlat);
                break;
            }
            case READY_WRIST_FLAT: {
                setWrist(kWristReadyFlatAngle);
                setElevator(kElevatorReadyFlat);
                break;
            }
            case READY_WRIST_LIP: {
                setWrist(kWristReadyFlatAngle);
                setElevator(kElevatorReadyLip);
                break;
            }
            // - - - Intaking States
            case INTAKE_BALL_WRIST_FLAT: {
                setWrist(kWristReadyFlatAngle);
                setElevator(kElevatorReadyFlat);
                handleBallIntake();
                break;
            }
            case INTAKE_BALL_WRIST_LIP: {
                setWrist(kWristReadyFlatAngle);
                setElevator(kElevatorReadyLip);
                handleBallIntake();
                break;
            }
            // - - - Move elevator and wrist to outtake balls
            case READY_BALL_LEVEL_1: {
                setElevator(kElevatorLevelOne);
                handleBallOuttake();
                break;
            }
            case READY_BALL_LEVEL_2: {
                setElevator(kElevatorLevelTwo);
                handleBallOuttake();
                break;
            }
            case READY_BALL_LEVEL_3: {
                setElevator(kElevatorLevelThree);
                handleBallOuttake();
                break;
            }
            // - - - Hatch intake from station
            case PREPARE_HATCH_INTAKE: {
                setElevator(kElevatorHatch);
                setWrist(kElevatorReadyFlat);
                if(checkHatchIntakeRequested()) {
                    setHatch(true);
                    currentState = SuperstructureState.READY_WRIST_LIP;
                } else {
                    setHatch(false);
                }
            }
            // - - - Move elevator and wrist to outtake hatches
            case READY_HATCH_LEVEL_1: {
                setElevator(kElevatorLevelOneH);
                setWrist(kWristReadyFlatAngle);
                handleHatchOuttake();
                break;
            }
            case READY_HATCH_LEVEL_2: {
                setElevator(kElevatorLevelTwoH);
                setWrist(kWristReadyFlatAngle);
                handleHatchOuttake();
                break;
            }
            case READY_HATCH_LEVEL_3: {
                setElevator(kElevatorLevelThreeH);
                setWrist(kWristReadyFlatAngle);
                handleHatchOuttake();
                break;
            }
        }
    }

    public void handleBallIntake() {
        if (checkRollerJammed()) {
            setRoller(0.0);
            currentState = SuperstructureState.READY_WRIST_RAISE;
        } else {
            setRoller(kRollerIntakePower);
        }
    }

    public void handleBallOuttake() {
        if (checkRequestOuttake()) {
            setWrist(kWristReadyOuttakeFlatAngle);
            setRoller(kRollerOuttakePower);
        } else if (finishRequestOuttake()) {
            setRoller(0.0);
            currentState = SuperstructureState.READY_WRIST_FLAT;
        } else {
            setWrist(kWristReadyOuttakeBallAngle);
        }
    }

    public void handleHatchOuttake() {
        if(checkRequestOuttake()) {
            setWrist(kWristReadyFlatAngle);
            setHatch(false);
        } else if(finishRequestOuttake()) {
            currentState = SuperstructureState.READY_WRIST_FLAT;
        } else {
            setWrist(kWristReadyFlatAngle);
        }
    }

    public void setWrist(double wrist_angle) {
        wrist_angle = wrist_angle - kWristStartOffset;
        superstructureSubsystem.wrist_des = wrist_angle;
    }

    public void setElevator(double elevator_pos) {
        superstructureSubsystem.elevator_des = elevator_pos;
    }

    public void setRoller(double roller_power) {
        superstructureSubsystem.roller_des = roller_power;
    }

    public void setHatch(boolean mode) {
        superstructureSubsystem.hatch_des = mode;
    }

    public boolean checkRollerJammed() {
        return false;
    }

    public boolean checkRequestOuttake() {
        return ControlBoard.getInstance().getOuttake();
    }

    //Check if the outtake button was just let go
    public boolean finishRequestOuttake() {
        return ControlBoard.getInstance().getOuttakeReleased();
    }

    public boolean checkHatchIntakeRequested() {
        //Essentially, check if the hatch intake button is not being pressed
        return !ControlBoard.getInstance().getIntakeHatch();
    }

}
