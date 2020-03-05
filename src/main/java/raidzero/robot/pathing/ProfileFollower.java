package raidzero.robot.pathing;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.Notifier;

import raidzero.pathgen.PathGenerator;
import raidzero.pathgen.PathPoint;
import raidzero.pathgen.Point;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.submodules.Drive.GearShift;
import raidzero.robot.utils.EncoderUtils;

public class ProfileFollower {

    /**
     * The states of the motion profile.
     */
    private enum State {
        FillPoints, WaitPoints, Run;
    };

    private BaseTalon leaderTalon;

    private boolean reversed;
    private GearShift gearShift;

    private boolean initRun;
    private State state;
    private MotionProfileStatus status;
    private SetValueMotionProfile setValue;

    /**
     * Runs periodically to push the trajectory points into the controller.
     */
    Notifier notifier = new Notifier(() -> {
        leaderTalon.processMotionProfileBuffer();
    });

    /**
     * Creates the profile follower.
     */
    public ProfileFollower(BaseTalon leader) {
        leaderTalon = leader;

        setValue = SetValueMotionProfile.Disable;
        status = new MotionProfileStatus();
        notifier.startPeriodic(0.001 * DriveConstants.TRANSMIT_PERIOD_MS);
        state = State.FillPoints;

        reversed = false;
        gearShift = GearShift.LOW;
    }

    /**
     * Starts the motion profile by generating & filling the points.
     *
     * @param points    the points to put in the path generator
     * @param cruiseVel the cruise velocity desired in in/100ms
     * @param tarAccel  the target acceleration desired in in/100ms/s
     */
    public void start(Point[] points, double cruiseVel, double tarAccel) {
        start(PathGenerator.generatePath(points, cruiseVel, tarAccel));
    }

    /**
     * Starts the motion profile by filling the points.
     *
     * @param points    the points to put in the path generator
     * @param cruiseVel the cruise velocity desired in in/100ms
     * @param tarAccel  the target acceleration desired in in/100ms/s
     */
    public void start(PathPoint[] pathPoints) {
        startFilling(pathPoints);
        initRun = true;
    }

    /**
     * Updates the state of the motion profile from the motor controller.
     * 
     * Note: Should be called periodically.
     */
    public void update() {
        switch (state) {
            case FillPoints:
                if (initRun) {
                    initRun = false;
                    setValue = SetValueMotionProfile.Disable;
                    state = State.WaitPoints;
                }
                break;
            case WaitPoints:
                leaderTalon.getMotionProfileStatus(status);
                if (status.btmBufferCnt > DriveConstants.MIN_POINTS_IN_TALON) {
                    setValue = SetValueMotionProfile.Enable;
                    state = State.Run;
                }
                break;
            case Run:
                leaderTalon.getMotionProfileStatus(status);
                if (status.activePointValid && status.isLast) {
                    setValue = SetValueMotionProfile.Hold;
                    state = State.FillPoints;
                }
                break;
        }
    }


    /**
     * Returns the output of the current set value. Should be passed to the set
     * method of the ultimate leader Talon.
     * 
     * @see SetValueMotionProfile
     * 
     * @return int
     */
    public int getOutput() {
        return setValue.value;
    }

    /**
     * Returns whether the current motion profile has finished executing.
     * 
     * @return if the profile is finished
     */
    public boolean isFinished() {
        return setValue == SetValueMotionProfile.Hold;
    }

    /**
     * Changes whether the path should be reversed or not. Must be called
     * before {@link #start(PathPoint[])}.
     *
     * @param reversed if the robot should move backwards
     */
    public void setReverse(boolean reversed) {
        this.reversed = reversed;
    }

    /**
     * Changes the gear while running this path. Must be called before
     * {@link #start(PathPoint[])}.
     *
     * @param gearShift gear of the drive
     */
    public void setGearShift(GearShift gearShift) {
        this.gearShift = gearShift;
    }

    /**
     * Clears the Motion profile buffer and resets state info.
     */
    public void reset() {
        leaderTalon.clearMotionProfileTrajectories();
        setValue = SetValueMotionProfile.Disable;
        state = State.FillPoints;
        initRun = false;
    }

    /**
     * Starts filling the buffer with trajectory points.
     *
     * @param waypoints the array of points created by the path generator
     */
    private void startFilling(PathPoint[] waypoints) {
        int reverse = reversed ? -1 : 1;
        // Clear under run error
        if (status.hasUnderrun) {
            leaderTalon.clearMotionProfileHasUnderrun();
        }

        // Clear the buffer just in case the robot is still running some points
        leaderTalon.clearMotionProfileTrajectories();

        for (int i = 0; i < waypoints.length; i++) {
            TrajectoryPoint tp = new TrajectoryPoint();
            tp.position = reverse * EncoderUtils.inchesToTicks(waypoints[i].position, gearShift);
            tp.velocity = reverse * EncoderUtils.inchesToTicks(waypoints[i].velocity, gearShift);
            // timeDur takes ms, but Pathpoint::time is in 100 ms
            tp.timeDur = (int) (waypoints[i].time * 100);
            // auxiliaryPos takes in units of 3600 ticks, but angle is in 360 degress
            tp.auxiliaryPos = waypoints[i].angle * 10;
            tp.useAuxPID = true;
            tp.profileSlotSelect0 = DriveConstants.PID_PRIMARY_SLOT;
            tp.profileSlotSelect1 = DriveConstants.PID_AUX_SLOT;
            tp.zeroPos = false;

            if (i == 0) {
                tp.zeroPos = true;
            }

            if (i == waypoints.length - 1) {
                tp.isLastPoint = true;
            }

            leaderTalon.pushMotionProfileTrajectory(tp);
        }
    }
}
