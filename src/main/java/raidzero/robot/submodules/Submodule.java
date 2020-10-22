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
     * Called once when the submodule is initialized.
     */
    public void onInit() {}

    /**
     * Design pattern for caching periodic reads to avoid hammering the 
     * CAN bus.
     */
    public void readPeriodicInputs() {}

    /**
     * Should do any logic needed to for the submodule.
     */
    public void update(double timestamp) {}

    /**
     * Design pattern for caching periodic writes to avoid hammering the 
     * CAN bus.
     */
    public void writePeriodicOutputs() {}

    /**
     * Stops the submodule.
     */
    public abstract void stop();

    /**
     * Resets the sensor(s) to zero.
     */
    public void zero() {}
}