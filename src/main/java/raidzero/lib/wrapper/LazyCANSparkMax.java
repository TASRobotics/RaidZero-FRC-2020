package raidzero.lib.wrapper;

import com.revrobotics.CANSparkMax;

public class LazyCANSparkMax extends CANSparkMax {
    
    protected double prevVal = 0;

    public LazyCANSparkMax(int deviceNumber, MotorType type) {
        super(deviceNumber, type);
    }

    public double getLastSet() {
        return prevVal;
    }

    @Override
    public void set(double value) {
        if (value != prevVal) {
            prevVal = value;
            super.set(value);
        }
    }
}