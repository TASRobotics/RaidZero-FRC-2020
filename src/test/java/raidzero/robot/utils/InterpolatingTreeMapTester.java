package raidzero.robot.utils;

import org.junit.*;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import java.util.concurrent.ThreadLocalRandom;

import raidzero.lib.util.InterpolatingTreeMap;
import raidzero.lib.util.InterpolatingDouble;

public class InterpolatingTreeMapTester {

    @Test
    public void isCorrectlyInterpolating() {
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> treeMap = 
            new InterpolatingTreeMap<>(5);
        treeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
        treeMap.put(new InterpolatingDouble(1.0), new InterpolatingDouble(2.0));
        treeMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(4.0));
        treeMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(6.0));
        treeMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(8.0));
        /*for (int i = 0; i < 50; ++i) {
            //double randNum = ThreadLocalRandom.current().nextDouble(0.0, 5.0);

            
        }*/
        assertEquals(
            treeMap.getInterpolated(new InterpolatingDouble(0.0)).value, 0.0, 0.001
        );
        assertEquals(
            treeMap.getInterpolated(new InterpolatingDouble(0.5)).value, 1.0, 0.001
        );
        assertEquals(
            treeMap.getInterpolated(new InterpolatingDouble(1.0 / 3.0)).value, 2.0 / 3.0, 0.001
        );
    }
}