package raidzero.robot.pathing;

import edu.wpi.first.wpilibj.Timer;

import raidzero.pathgen.PathGenerator;
import raidzero.pathgen.PathPoint;
import raidzero.pathgen.Point;
import raidzero.robot.Constants.DriveConstants;

public class Path {

    private Point[] points;
    private PathPoint[] pathPoints;
    private double cruiseVel;
    private double targetAccel;
    private boolean reversed;

    /**
     * Constructs a Path with default cruise velocity and target acceleration.
     * 
     * Note: The motion profile is generated in the constructor.
     * 
     * @param points   waypoints in the path
     * @param reversed whether to follow the path reversed
     */
    public Path(Point[] points, boolean reversed) {
        this(points, reversed, DriveConstants.DEFAULT_CRUISE_VELOCITY,
                DriveConstants.DEFAULT_TARGET_ACCELERATION);
    }

    /**
     * Constructs a Path with custom cruise velocity and target acceleration.
     * 
     * Note: The motion profile is generated in the constructor.
     * 
     * @param points      waypoints in the path
     * @param reversed    whether to follow the path reversed
     * @param cruiseVel   the target cruise velocity in in/100ms
     * @param targetAccel the target acceleration in in/100ms/s
     */
    public Path(Point[] points, boolean reversed, double cruiseVel, double targetAccel) {
        this.points = points;
        this.reversed = reversed;
        this.cruiseVel = cruiseVel;
        this.targetAccel = targetAccel;

        double startTime = Timer.getFPGATimestamp();
        pathPoints = PathGenerator.generatePath(points, cruiseVel, targetAccel);
        System.out.println("PathGenerator: It took " + (Timer.getFPGATimestamp() - startTime)
                + "s to generate a path!");
    }

    public double getInitialAngle() {
        return (getFirstPoint().angle.orElse(0.0) + (isReversed() ? 180.0 : 0.0)) % 360.0;
    }

    /**
     * Returns the waypoints of the path.
     * 
     * @return array of waypoints
     */
    public Point[] getPoints() {
        return points;
    }

    /**
     * Returns the first waypoint of the path.
     * 
     * @return first point
     */
    public Point getFirstPoint() {
        return points[0];
    }

    /**
     * Returns the last waypoint of the path.
     * 
     * @return last point
     */
    public Point getLastPoint() {
        return points[points.length - 1];
    }

    /**
     * Returns the path points for the motion profile.
     * 
     * @return array of path points
     */
    public PathPoint[] getPathPoints() {
        return pathPoints;
    }

    /**
     * Returns whether the path should be followed reversed.
     * 
     * @return reverse while driving
     */
    public boolean isReversed() {
        return reversed;
    }

    /**
     * Returns the target cruise velocity of the robot.
     * 
     * @return cruise velocity in in/100ms
     */
    public double getCruiseVelocity() {
        return cruiseVel;
    }

    /**
     * Returns the target acceleration of the robot.
     * 
     * @return target acceleration in in/100ms/s
     */
    public double getTargetAcceleration() {
        return targetAccel;
    }
}
