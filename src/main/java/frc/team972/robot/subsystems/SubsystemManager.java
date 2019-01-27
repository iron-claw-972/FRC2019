package frc.team972.robot.subsystems;

import frc.team972.robot.loops.ILooper;
import frc.team972.robot.loops.Loop;
import frc.team972.robot.loops.Looper;

import java.util.ArrayList;
import java.util.List;

public class SubsystemManager implements ILooper {

    private final List<Subsystem> mAllSubsystems;
    private List<Loop> mLoops = new ArrayList<>();

    public SubsystemManager(List<Subsystem> allSubsystems) {
        mAllSubsystems = allSubsystems;
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

    private class EnabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {
            for (Loop l : mLoops) {
                l.onStart(timestamp);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            for (Subsystem s : mAllSubsystems) {
                s.fastPeriodic();
            }
            for (Loop l : mLoops) {
                l.onLoop(timestamp);
            }
        }

        @Override
        public void onStop(double timestamp) {
            for (Loop l : mLoops) {
                l.onStop(timestamp);
            }
        }
    }

    private class DisabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            for (Subsystem s : mAllSubsystems) {
                s.fastPeriodic();
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach((s) -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }
}