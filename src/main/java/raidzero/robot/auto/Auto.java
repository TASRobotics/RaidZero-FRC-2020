package raidzero.robot.auto;

import raidzero.robot.components.Components;

import java.util.LinkedList;
import java.util.Queue;

import com.ctre.phoenix.motion.SetValueMotionProfile;

import raidzero.pathgen.Point;

public class Auto {

    private static final double CRUISE_VELOCITY = 10;
    private static final double TARGET_ACCELERATION = 20;
    
    private static MotionProfile profile;
    private static boolean isRunning;

    private static Queue<PathInfo> pathQueue = new LinkedList<>();

    private static Point[] test = {
        new Point(0, 0, 0),
        new Point(50, 50, 0),
        new Point(100, 100, 0)
    };

    /**
     * Initialize the auto-specific components.
     *
     * <p>Should be called when the robot starts up.
     */
    public static void initialize() {
        profile = new MotionProfile(Components.getBase().getRightMotor(),
            Components.getBase().getLeftMotor(), Components.getBase().getPigeon());
        isRunning = false;
    }

    /**
     * Configures the components for use in autonomous mode.
     *
     * <p>This should be called once every time the robot is switched to autonomous mode, before
     * calling {@link #run()}.
     */
    public static void setup() {
        isRunning = false;

        // Read waypoints
        pathQueue.add(new PathInfo(
            test, true
        ));
    }

    /**
     * Runs the autonomous code.
     *
     * <p>This should be called repeatedly during autonomous mode.
     */
    public static void run() {
        if (!isRunning) {
            if (pathQueue.size() > 0) {
                // Reset encoders & MP
                Components.getBase().zeroSensors();
                profile.reset();

                PathInfo path = pathQueue.poll();
                profile.setReverse(path.isReversed());
                profile.start(path.getPoints(), CRUISE_VELOCITY, TARGET_ACCELERATION);
                isRunning = true;
            }
        } else {
            profile.controlMP();
            profile.move();
            if (profile.getSetValue() == SetValueMotionProfile.Hold) {
                isRunning = false;
            }
        }
    }

    /**
     * Run code for disabled periodic
     */
    public static void disabled() {

    }
}
