package raidzero.lib.wrapper;

import com.ctre.phoenix.sensors.PigeonIMU;

public class Pigeon extends PigeonIMU {

    // private double[] yawPitchRoll = new double[3];

    public Pigeon(int deviceNumber) {
        super(deviceNumber);
    }

    /**
     * Returns the heading of the robot in form required for odometry.
     *
     * @return the robot's heading in degrees, from 180 to -180 with a
     *         positive value for left turn (counter-clockwise).
     */
    public double getHeading() {
        // getYawPitchRoll(yawPitchRoll);
        return Math.IEEEremainder(getFusedHeading(), 360.0);
    }
}