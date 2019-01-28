package frc.team972.robot.subsystems;

import frc.team972.robot.loops.Loop;
import frc.team972.robot.loops.Looper;

import java.util.ArrayList;
import java.util.List;

public class SubsystemManager {

    private final List<Subsystem> mAllSubsystems;
    private List<Loop> mLoops = new ArrayList<>();

    public SubsystemManager(List<Subsystem> allSubsystems) {
        mAllSubsystems = allSubsystems;
        for (Subsystem subsystem : allSubsystems) {
            SubsystemLoop subsystemLoop = new SubsystemLoop(subsystem, subsystem.getClass().getName());
            subsystemLoop.name = subsystem.toString();
            mLoops.add(subsystemLoop);
        }
    }

    public void outputToSmartDashboard() {
        mAllSubsystems.forEach((s) -> s.outputTelemetry());
    }

    public void slowPeriodic() {
        mAllSubsystems.forEach((s) -> s.slowPeriodic());
    }

    public void writeToLog() {
        mAllSubsystems.forEach((s) -> s.writeToLog());
    }

    public void stop() {
        mAllSubsystems.forEach((s) -> s.stop());
    }

    private class SubsystemLoop implements Loop {
        Subsystem subsystem;
        String name = "";

        public SubsystemLoop(Subsystem subsystem, String name) {
            this.subsystem = subsystem;
            this.name = name;
        }

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            subsystem.fastPeriodic();
        }

        @Override
        public void onStop(double timestamp) {
            subsystem.stop();
        }

        @Override
        public String getName() {
            return name;
        }
    }

    public void registerLoops(Looper mLooper) {
        for (Loop subsystemLoop : mLoops) {
            mLooper.register(subsystemLoop);
        }
    }
}