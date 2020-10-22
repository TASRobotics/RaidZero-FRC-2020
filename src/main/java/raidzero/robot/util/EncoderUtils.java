package raidzero.robot.util;

import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.submodules.Drive.GearShift;

public class EncoderUtils {

    public static double ticksToInches(double ticks, GearShift gear) {
        if (gear == GearShift.HIGH) {
            return ticks / DriveConstants.SENSOR_UNITS_PER_INCH_HIGH_GEAR;
        }
        return ticks / DriveConstants.SENSOR_UNITS_PER_INCH_LOW_GEAR;
    }

    public static double inchesToTicks(double inches, GearShift gear) {
        if (gear == GearShift.HIGH) {
            return inches * DriveConstants.SENSOR_UNITS_PER_INCH_HIGH_GEAR;
        }
        return inches * DriveConstants.SENSOR_UNITS_PER_INCH_LOW_GEAR;
    }
}