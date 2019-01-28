package frc.team972.robot.loops;

import java.util.ArrayList;
import java.util.List;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper implements ILooper {
    public final double kPeriod = 0.01;

    private boolean running_;

    private final List<Loop> loops_;
    private final Object taskRunningLock_ = new Object();
    private double timestamp_ = 0;
    private double dt_ = 0;

    private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() {
        @Override
        public void runCrashTracked() {
            synchronized (taskRunningLock_) {
                if (running_) {
                    //SPAWN NEW THREAD
                    for (Loop loop : loops_) {
                        Thread thread = new Thread() {
                            public void run() {
                                while (running_) {
                                    double now = System.currentTimeMillis();
                                    loop.onLoop(now);
                                    dt_ = now - timestamp_;
                                    timestamp_ = now;
                                }
                            }
                        };
                        System.out.println("Spawning new Thread: " + loop.getName());
                        thread.start();
                    }
                }
            }
        }
    };

    public Looper() {
        running_ = false;
        loops_ = new ArrayList<>();
    }

    @Override
    public synchronized void register(Loop loop) {
        synchronized (taskRunningLock_) {
            loops_.add(loop);
        }
    }

    public synchronized void start() {
        if (!running_) {
            System.out.println("Starting loops");
            synchronized (taskRunningLock_) {
                timestamp_ = System.currentTimeMillis();
                for (Loop loop : loops_) {
                    loop.onStart(timestamp_);
                }
                running_ = true;
            }
            runnable_.run();
        }
    }

    public synchronized void stop() {
        if (running_) {
            System.out.println("Stopping loops");
            synchronized (taskRunningLock_) {
                running_ = false;
                timestamp_ = System.currentTimeMillis();
                for (Loop loop : loops_) {
                    System.out.println("Stopping " + loop);
                    loop.onStop(timestamp_);
                }
            }
        }
    }
}
