package raidzero.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * A boolean that only returns true if the supplied boolean stays true for
 * a specified duration.
 */
public class TimerBoolean {

    private double duration = 0.0;
    private boolean previousBoolean = false;

    private double startTime = 0.0;
    private boolean started = false;

    /**
     * Constructs a TimerBoolean with the specified duration.
     * 
     * @param duration time to wait for in seconds
     */
    public TimerBoolean(double duration) {
        this.duration = duration;
    }

    public void reset() {
        previousBoolean = false;
        startTime = 0.0;
        started = false;
    }

    public void update(boolean currentBoolean) {
        if (!previousBoolean && currentBoolean) { // If just switched to true
            started = true;
            startTime = Timer.getFPGATimestamp();
        } else if (!currentBoolean && previousBoolean) { // If was true but turned false
            started = false;
        }
        previousBoolean = currentBoolean;
    }

    public boolean hasDurationPassed() {
        return started && (Timer.getFPGATimestamp() - startTime >= duration);
    }
}