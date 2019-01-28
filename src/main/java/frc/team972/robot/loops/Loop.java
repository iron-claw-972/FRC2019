package frc.team972.robot.loops;

public interface Loop {
    void onStart(double timestamp);

    void onLoop(double timestamp);

    void onStop(double timestamp);

    String getName();
}
