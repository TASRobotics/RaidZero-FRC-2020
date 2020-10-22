package raidzero.lib.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * A boolean that only returns true if the supplied boolean stays true for
 * a specified duration.
 */
public class TimedBoolean {

    private double duration = 0.0;
    private boolean previousBoolean = false;
    private Timer timer = new Timer();

    /**
     * Constructs a TimerBoolean with the specified duration.
     * 
     * @param duration time to wait for in seconds
     */
    public TimedBoolean(double duration) {
        this.duration = duration;
    }

    public void reset() {
        previousBoolean = false;
        timer.reset();
    }

    public void update(boolean currentBoolean) {
        if (!previousBoolean && currentBoolean) { // If just switched to true
            timer.reset();
            timer.start();
        }
        previousBoolean = currentBoolean;
    }

    public boolean hasDurationPassed() {
        return previousBoolean && timer.get() >= duration;
    }
}