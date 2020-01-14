package raidzero.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.SubmoduleManager;

public class PeriodicExecutor {

    private boolean isRunning;
    private Notifier notifier;
    private final Object mutex = new Object();

    private Runnable task = new Runnable() {
        @Override
        public void run() {
            synchronized (mutex) {
                if (isRunning) {
                    SubmoduleManager.getInstance().onLoop(Timer.getFPGATimestamp());
                }
            }
        }
    };

    public PeriodicExecutor() {
        isRunning = false;
        notifier = new Notifier(task);
    }

    public synchronized void start() {
        if (!isRunning) {
            System.out.println("Starting PeriodicExecutor");

            synchronized (mutex) {
                SubmoduleManager.getInstance().onStart(Timer.getFPGATimestamp());
                isRunning = true;
            }
            notifier.startPeriodic(0.01);
        }
    }

    public synchronized void stop() {
        if (isRunning) {
            System.out.println("Stopping PeriodicExecutor");
            notifier.stop();

            synchronized (mutex) {
                SubmoduleManager.getInstance().onStop(Timer.getFPGATimestamp());
                isRunning = false;
            }
        }
    }
}