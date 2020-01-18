package raidzero.robot.wrappers;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class InactiveDoubleSolenoid extends DoubleSolenoid{
    protected boolean active = false;

    public InactiveDoubleSolenoid(int FwdSolenoid, int RvrseSolenoid) {
        super(FwdSolenoid, RvrseSolenoid);
    }
 
    public void setState(boolean state) {
      active = state;
    }
  
    @Override
    public void set(Value value) {
      if(!active) {
        return;
      }
      super.set(value);
    }
}