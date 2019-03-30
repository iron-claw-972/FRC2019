package frc.team972.robot.statemachines;

import frc.team972.robot.Constants;
import frc.team972.robot.subsystems.ElevatorSubsystem;
import frc.team972.robot.subsystems.SuperstructureSubsystem;
import frc.team972.robot.subsystems.WristSubsystem;
import frc.team972.robot.teleop.ControlBoard;

public class SuperstructureStateMachine {

    public static final double kWristStartOffset = 19.0;
    public static final double kWristReadyRaiseAngle = 90;
    public static final double kWristReadyOuttakeBallAngle = 90 + 25.0;
    public static final double kWristReadyOuttakeFlatAngle = 150;
    public static final double kWristReadyFlatAngle = 179.0;


    public static final double kElevatorReadyFlat = 0.0;
    public static final double kElevatorReadyLip = 0.25;
    public static final double kElevatorReadyHatch = 0.3;
    public static final double kElevatorLevelOne = 0.805;
    public static final double kElevatorLevelTwo = 1.6;
    public static final double kElevatorLevelThree = 2.0;

    public static final double kElevatorHatch = 0.20;
    public static final double kElevatorLevelOneH = 0.21;
    public static final double kElevatorLevelTwoH = 0.22;
    public static final double kElevatorLevelThreeH = 1.6;


    public static final double kRollerIntakePower = -1.0;
    public static final double kRollerOuttakePower = 1.0;

    public SuperstructureState currentState = SuperstructureState.READY_WRIST_RAISE;
    SuperstructureSubsystem superstructureSubsystem;

    public SuperstructureStateMachine(SuperstructureSubsystem superstructureSubsystem) {
        this.superstructureSubsystem = superstructureSubsystem;
    }

    public void update() {
        //System.out.println(currentState);
        switch (currentState) {
            // - - - Ready States
            case READY_WRIST_RAISE: {
                setWrist(kWristReadyRaiseAngle);
                setElevator(kElevatorReadyFlat);
                setHatch(true);
                setRoller(0.0);
                break;
            }
            case STOW_WRIST: {
                setWrist(45);
                setElevator(kElevatorReadyFlat);
                setHatch(false);
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
            case READY_WRIST_LIP_HATCH: {
                setWrist(kWristReadyFlatAngle);
                setElevator(kElevatorReadyHatch);
                break;
            }
            // - - - Intaking States
            case INTAKE_BALL_WRIST_FLAT: {
                setWrist(kWristReadyFlatAngle);
                setElevator(kElevatorReadyFlat-0.01);
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
                    currentState = SuperstructureState.READY_WRIST_LIP_HATCH;
                    setHatch(true);
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
        setHatch(true);
        if (checkRollerJammed() || finishRequestBallIntake()) {
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
            currentState = SuperstructureState.READY_WRIST_RAISE;
        } else {
            setWrist(kWristReadyOuttakeBallAngle);
        }
    }

    public void handleHatchOuttake() {
        if(checkRequestOuttake()) {
            setWrist(kWristReadyFlatAngle);
            setHatch(false);
        } else if(finishRequestOuttake()) {
            currentState = SuperstructureState.READY_WRIST_RAISE;
        } else {
            setWrist(kWristReadyFlatAngle);
        }
    }

    public void setWrist(double wrist_angle) {
        //wrist_angle = wrist_angle - kWristStartOffset;
        superstructureSubsystem.wrist_des = Math.toRadians(wrist_angle);
    }

    public void setElevator(double elevator_pos) {
        //System.out.println(elevator_pos);
        superstructureSubsystem.elevator_des = elevator_pos;
    }

    public void setRoller(double roller_power) {
        superstructureSubsystem.roller_des = roller_power - 0.1;
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

    public boolean finishRequestBallIntake() {
        return ControlBoard.getInstance().getBallIntakeReleased();
    }

    public boolean checkHatchIntakeRequested() {
        //Essentially, check if the hatch intake button is not being pressed
        return !ControlBoard.getInstance().getIntakeHatch();
    }

}
