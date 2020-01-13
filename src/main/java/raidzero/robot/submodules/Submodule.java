package raidzero.robot.submodules;

/**
 *  An abstract low level parent class for all subsystems. Contains common methods to be used for the bot
 */
public abstract class Submodule {
    // The active variable is used to disable a subcomponent for unit testing or other various reasons
    public Boolean active = true;
    
    /**
     * Initiates the variables and settings needed for the object
     */
    public void init() {}

    /**
     * Method that will add other methods to the call stack to run the submodule
     */
    public void run() {}

    /**
     * Resets the sensor(s) to zero
     */
    public void zero() {}
}