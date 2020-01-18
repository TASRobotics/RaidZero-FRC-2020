package raidzero.robot.wrappers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LazyTalonSRX extends TalonSRX {
    protected double prevVal = 0;
    protected ControlMode prevControlMode = null;

    public LazyTalonSRX(int deviceNumber) {
        super(deviceNumber);
    }

    public double getprevVal() {
        return prevVal;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != prevVal || mode != prevControlMode) {
            prevVal = value;
            prevControlMode = mode;
            super.set(mode, value);
        }
    }
}