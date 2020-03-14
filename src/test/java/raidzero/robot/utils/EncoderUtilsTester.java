package raidzero.robot.utils;

import org.junit.*;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import java.util.concurrent.ThreadLocalRandom;

import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.submodules.Drive.GearShift;
import raidzero.robot.util.EncoderUtils;

public class EncoderUtilsTester {

    @Test
    public void isCorrectConversion() {
        for (int i = 0; i < 50; ++i) {
            double randNum = ThreadLocalRandom.current().nextDouble(1, 10);

            assertEquals(
                EncoderUtils.inchesToTicks(randNum, GearShift.LOW),
                randNum * DriveConstants.SENSOR_UNITS_PER_INCH_LOW_GEAR, 0.0001);
            assertEquals(
                EncoderUtils.inchesToTicks(randNum, GearShift.HIGH),
                randNum * DriveConstants.SENSOR_UNITS_PER_INCH_HIGH_GEAR, 0.0001);

            assertEquals(
                EncoderUtils.metersToTicks(randNum, GearShift.LOW),
                randNum / 0.0254 * DriveConstants.SENSOR_UNITS_PER_INCH_LOW_GEAR, 0.0001);
            assertEquals(
                EncoderUtils.metersToTicks(randNum, GearShift.HIGH),
                randNum / 0.0254 * DriveConstants.SENSOR_UNITS_PER_INCH_HIGH_GEAR, 0.0001);
    
            assertEquals(
                EncoderUtils.metersPerSecToTicksPer100ms(randNum, GearShift.LOW),
                randNum / 0.0254 * DriveConstants.SENSOR_UNITS_PER_INCH_LOW_GEAR / 10, 0.0001);
            assertEquals(
                EncoderUtils.metersPerSecToTicksPer100ms(randNum, GearShift.HIGH),
                randNum / 0.0254 * DriveConstants.SENSOR_UNITS_PER_INCH_HIGH_GEAR / 10, 0.0001);
        }
    }
}