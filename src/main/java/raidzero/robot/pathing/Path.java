package raidzero.robot.pathing;

import edu.wpi.first.wpilibj.Timer;

import raidzero.pathgen.PathGenerator;
import raidzero.pathgen.PathPoint;
import raidzero.pathgen.Point;
import raidzero.robot.Constants;

public class Path {

    private Point[] points;
    private PathPoint[] pathPoints;
    private double cruiseVel;
    private double targetAccel;
    private boolean reversed;

    public Path(Point[] points, boolean reversed) {
        this(points, reversed, Constants.DEFAULT_CRUISE_VELOCITY, 
            Constants.DEFAULT_TARGET_ACCELERATION);
    }

    public Path(Point[] points, boolean reversed, double cruiseVel, double targetAccel) {
        this.points = points;
        this.reversed = reversed;
        this.cruiseVel = cruiseVel;
        this.targetAccel = targetAccel;

        double startTime = Timer.getFPGATimestamp();
        pathPoints = PathGenerator.generatePath(points, cruiseVel, 
            targetAccel);
        System.out.println(
            "PathGenerator: It took " + (Timer.getFPGATimestamp() - startTime) + "s to generate a path!");
    }

    public Point[] getPoints() {
        return points;
    }

    public PathPoint[] getPathPoints() {
        return pathPoints;
    }

    public boolean isReversed() {
        return reversed;
    }
    
    public double getCruiseVelocity() {
        return cruiseVel;
    }

    public double getTargetAcceleration() {
        return targetAccel;
    }
}