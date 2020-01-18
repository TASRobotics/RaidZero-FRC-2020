package raidzero.robot.pathing;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Notifier;

import raidzero.pathgen.PathGenerator;
import raidzero.pathgen.PathPoint;
import raidzero.pathgen.Point;
import raidzero.robot.Constants;

public class ProfileFollower {

    /**
     * The states of controlling the motion profile
     */
    private enum State {
        FillPoints, WaitPoints, Run;
    };

    private TalonFX leaderTalon;

    private boolean reversed;
    private boolean initRun;
    private State state;
    private MotionProfileStatus status;
    private SetValueMotionProfile setValue;

    Notifier notifier = new Notifier(() -> {
        leaderTalon.processMotionProfileBuffer();
    });

    /**
     * Creates the profile follower.
     */
    public ProfileFollower(TalonFX leader) {
        leaderTalon = leader;

        setValue = SetValueMotionProfile.Disable;
        status = new MotionProfileStatus();
        notifier.startPeriodic(0.001 * Constants.TRANSMIT_PERIOD_MS);
        state = State.FillPoints;
        reversed = false;
    }

    /**
     * Starts the motion profile by generating & filling the points.
     *
     * @param points the points to put in the path generator
     * @param cruiseVel the cruise velocity desired in in/100ms
     * @param tarAccel the target acceleration desired in in/100ms/s
     */
    public void start(Point[] points, double cruiseVel, double tarAccel) {
        start(PathGenerator.generatePath(points, cruiseVel, tarAccel));
    }

    /**
     * Starts the motion profile by filling the points.
     *
     * @param points the points to put in the path generator
     * @param cruiseVel the cruise velocity desired in in/100ms
     * @param tarAccel the target acceleration desired in in/100ms/s
     */
    public void start(PathPoint[] pathPoints) {
        startFilling(pathPoints);
        initRun = true;
    }

    /**
     * Call periodically to control states of the motion profile.
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
                if (status.btmBufferCnt > Constants.MIN_POINTS_IN_TALON) {
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
     * Moves the base in motion profile arc mode.
     *
     * <p>This should be periodically called.
     */
    public int getOutput() {
        return setValue.value;
    }

    public boolean isFinished() {
        return setValue == SetValueMotionProfile.Hold;
    }

    /**
     * Set the path to go reversed or not
     *
     * @param reversed true if want bot to go backwards, else false
     */
    public void setReverse(boolean reversed) {
        this.reversed = reversed;
    }

    /**
     * Clears the Motion profile buffer and resets state info
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
            tp.position = waypoints[i].position * reverse * Constants.SENSOR_UNITS_PER_INCH;
            tp.velocity = waypoints[i].velocity * reverse * Constants.SENSOR_UNITS_PER_INCH;
            // timeDur takes ms, but Pathpoint::time is in 100 ms
            tp.timeDur = (int) (waypoints[i].time * 100);
            tp.auxiliaryPos = waypoints[i].angle * 10;
            tp.useAuxPID = true;
            tp.profileSlotSelect0 = Constants.PID_PRIMARY_SLOT;
            tp.profileSlotSelect1 = Constants.PID_AUX_SLOT;
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
