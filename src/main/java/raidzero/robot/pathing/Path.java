package raidzero.robot.pathing;

import raidzero.pathgen.Point;
import raidzero.robot.Constants;

public class Path {

    private Point[] points;
    private double cruiseVel = Constants.DEFAULT_CRUISE_VELOCITY;
    private double targetAccel = Constants.DEFAULT_TARGET_ACCELERATION;
    private boolean reversed;

    public Path(Point[] points, boolean reversed) {
        this.points = points;
        this.reversed = reversed;
    }

    public Path(Point[] points, boolean reversed, double cruiseVel, double targetAccel) {
        this.points = points;
        this.reversed = reversed;
        this.cruiseVel = cruiseVel;
        this.targetAccel = targetAccel;
    }

    public Point[] getPoints() {
        return points;
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