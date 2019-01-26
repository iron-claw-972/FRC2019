package frc.team972.robot;

public class RobotState {
    public boolean outputs_enabled = false;
    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }

    //TODO: Implement.

}
