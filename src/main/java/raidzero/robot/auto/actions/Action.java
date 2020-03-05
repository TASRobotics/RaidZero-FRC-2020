package raidzero.robot.auto.actions;

public interface Action {
    
    /**
     * Returns whether the action is finished or not.
     * 
     * @return if the action is finished
     */
    boolean isFinished();

    /**
     * Called AutoSequence iteratively until isFinished returns true. 
     * Iterative logic lives in this method.
     */
    void update();

    /**
     * Runs code once when the action finishes, usually for clean up.
     */
    void done();

    /**
     * Runs code once when the action is started for set up.
     */
    void start();
}