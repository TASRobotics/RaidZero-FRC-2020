package raidzero.robot.submodules;

/**
 * An abstract low level parent class for all subsystems.
 */
public abstract class Submodule {

    // Used to disable a submodule for unit testing or other reasons.
    public boolean isEnabled = true;

    public void onStart(double timestamp) {}

    /**
     * Reads cached inputs & calculate outputs.
     */
    public void update(double timestamp) {}
    
    /**
     * Runs components in the submodule that have continuously changing inputs.
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

    /**
     * Enables this submodule.
     * 
     * @param enabled
     */
    public void setEnabled(boolean enabled) {
        isEnabled = enabled;
    }

    /**
     * Returns whether the submodule is enabled or not.
     */
    public boolean isEnabled() {
        return isEnabled;
    }
}