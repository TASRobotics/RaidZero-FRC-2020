package raidzero.robot.utils;

import raidzero.robot.Constants;
import raidzero.robot.submodules.Drive.GearShift;

public class EncoderUtils {

    public static double ticksToInches(double ticks, GearShift gear) {
        if (gear == GearShift.HIGH) {
            return ticks / Constants.SENSOR_UNITS_PER_INCH_HIGH_GEAR;
        }
        return ticks / Constants.SENSOR_UNITS_PER_INCH_LOW_GEAR;
    }

    public static double inchesToTicks(double inches, GearShift gear) {
        if (gear == GearShift.HIGH) {
            return inches * Constants.SENSOR_UNITS_PER_INCH_HIGH_GEAR;
        }
        return inches * Constants.SENSOR_UNITS_PER_INCH_LOW_GEAR;
    }
}