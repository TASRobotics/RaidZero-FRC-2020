package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import raidzero.robot.Constants.ClimbConstants;
import raidzero.robot.wrappers.LazyTalonSRX;

public class Climb extends Submodule {

    private static Climb instance = null;
    
    private LazyTalonSRX climbMotor;
    private PWM servo;

    private boolean unlocked = false;
    private double outputOpenLoop = 0.0;
    private int serve = -1;
    
    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    private Climb() {}
    
    @Override
    public void onInit() {
        servo = new PWM(9);
        climbMotor = new LazyTalonSRX(ClimbConstants.MOTOR_ID);
        climbMotor.configFactoryDefault();
        climbMotor.setNeutralMode(ClimbConstants.NEUTRAL_MODE);
        climbMotor.setInverted(ClimbConstants.INVERSION);
    }

    @Override
    public void run() {
        if (!unlocked) {
            serve = 1;
            outputOpenLoop = 0.0;
        }
        servo.setPosition(serve);
        climbMotor.set(ControlMode.PercentOutput, outputOpenLoop);
    }

    @Override
    public void stop() {
        outputOpenLoop = 0.0;
        climbMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Unlocks the climb.
     */
    public void unlock() {
        unlocked = true;
    }

    public void openServo() {
        serve = -1;
    }

    public void closeServo() {
        serve = 1;  
    }

    /**
     * Climbs using percent output.
     * 
     * @param percentOutput percent output in [-1, 1]
     */
    public void climb(double percentOutput) {
        outputOpenLoop = percentOutput;
    }
}