package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.submodules.Drive;

/**
 * Action for driving the base using open-loop control.
 */
public class DriveOpenLoop implements Action {

    private static final Drive drive = Drive.getInstance();

    private double leftOutput = 0.0;
    private double rightOutput = 0.0;
    private double duration = 0.0;

    private double startTime;

    /**
     * Constructs a DriveOpenLoop action.
     * 
     * @param left left percent output in [-1, 1]
     * @param right right percent output in [-1, 1]
     * @param duration drive duration in seconds
     */
    public DriveOpenLoop(double left, double right, double duration) {
        leftOutput = left;
        rightOutput = right;
        this.duration = duration;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= duration;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        startTime = Timer.getFPGATimestamp();

        drive.setOpenLoop();
        drive.tank(leftOutput, rightOutput);
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        drive.stop();
    }
}