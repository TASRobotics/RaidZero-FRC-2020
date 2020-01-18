package raidzero.pathgen;

import java.util.OptionalDouble;

/**
 * Simple tuple-like data class with x y coordinates and optional angle.
 */
public class Point {

    /**
     * The x-coordinate. (+x is forward in the robot's frame)
     */
    public final double x;

    /**
     * The y-coordinate. (+y is to the left in the robot's frame)
     */
    public final double y;

    /**
     * The angle in degrees, if provided.
     */
    public final OptionalDouble angle;

    /**
     * Constructs a Point object.
     *
     * @param x the x-coordinate
     * @param y the y-coordinate
     */
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
        this.angle = OptionalDouble.empty();
    }

    /**
     * Constructs a Point object.
     *
     * @param x the x-coordinate
     * @param y the y-coordinate
     * @param a the angle in degrees
     */
    public Point(double x, double y, double a) {
        this.x = x;
        this.y = y;
        this.angle = OptionalDouble.of(a);
    }
}
