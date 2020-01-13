package raidzero.robot.auto;

import raidzero.pathgen.Point;

public class PathInfo {

    private Point[] points;
    private boolean reversed;

    public PathInfo(Point[] points, boolean reversed) {
        this.points = points;
        this.reversed = reversed;
    }

    public Point[] getPoints() {
        return points;
    }

    public boolean isReversed() {
        return reversed;
    }
}