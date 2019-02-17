package frc.team972.robot.loops;

import frc.team972.robot.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper implements ILooper {
    private boolean running_;

    private final List<Loop> loops_;
    private final Object taskRunningLock_ = new Object();
    private double timestamp_ = 0;

    private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() {
        @Override
        public void runCrashTracked() {
            synchronized (taskRunningLock_) {
                if (running_) {
                    //SPAWN NEW THREAD
                    for (Loop loop : loops_) {
                        Thread thread = new Thread() {
                            public void run() {
                                try {
                                    while (running_) {
                                        long now = System.currentTimeMillis();
                                        loop.onLoop(now);
                                        long timestamp_ = System.currentTimeMillis();
                                        long dt_ = timestamp_ - now;
                                        long remain_time = (long) (Constants.dt * 1000) - dt_;
                                        if (remain_time < 0) {
                                            //System.out.println("MAJOR WARNING, RT LOOP CAN NOT KEEP UP!!! " + currentThread().getName());
                                        } else {
                                            Thread.sleep(remain_time);
                                        }
                                    }
                                } catch (Exception e) {
                                    e.printStackTrace();
                                }
                            }
                        };
                        System.out.println("Spawning new Thread: " + loop.getName());
                        thread.setPriority(Thread.MAX_PRIORITY); // real time priority
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
