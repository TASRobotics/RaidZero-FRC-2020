package raidzero.robot.utils;

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

    public static double ticksToMeters(double ticks, GearShift gear) {
        return ticksToInches(ticks, gear) * DriveConstants.METERS_PER_INCH;
    }

    public static double metersToTicks(double meters, GearShift gear) {
        return inchesToTicks(meters * DriveConstants.INCHES_PER_METER, gear);
    }

    public static double ticksPer100msToMetersPerSec(double ticksPer100ms, GearShift gear) {
        return ticksToMeters(ticksPer100ms * 10, gear);
    }

    public static double metersPerSecToTicksPer100ms(double metersPerSec, GearShift gear) {
        return metersToTicks(metersPerSec, gear) * 0.1;
    }
}