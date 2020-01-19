package raidzero.robot.submodules;

/**
 * An abstract low level parent class for all subsystems.
 */
public abstract class Submodule {

    /**
     * Called at the start of autonomous or teleop.
     * 
     * @param timestamp
     */
    public void onStart(double timestamp) {}

    /**
     * Reads cached inputs & calculate outputs.
     * 
     * @param timestamp
     */
    public void update(double timestamp) {}
    
    /**
     * Runs components in the submodule that have continuously changing 
     * inputs.
     */
    public void run() {}

    /**
     * Stops the submodule.
     */
    public abstract void stop();

    /**
     * Resets the sensor(s) to zero.
     */
    public void zero() {}
}