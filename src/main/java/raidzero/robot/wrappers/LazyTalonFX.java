package raidzero.robot.wrappers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class LazyTalonFX extends TalonFX {
    
    protected double prevVal = 0;
    protected ControlMode prevControlMode = null;

    public LazyTalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    public double getLastSet() {
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