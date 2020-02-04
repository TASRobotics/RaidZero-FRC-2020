package raidzero.robot.submodules;

import raidzero.robot.wrappers.LazyCANSparkMax;
import raidzero.robot.wrappers.LazyTalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import raidzero.robot.Constants;

public class Shooter extends Submodule {

    private static LazyTalonFX motor;
    private static Shooter instance = null;

    private static double shooterSpeed = 0.0;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        /**
         * Epididymis Init
         */
        motor = new LazyTalonFX(8);
        motor.setInverted(Constants.shooterInvert);
        motor.setNeutralMode(NeutralMode.Coast);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; 
        config.primaryPID.selectedFeedbackCoefficient = 1.0;
        config.slot0.kF = Constants.shooterF;
        config.slot0.kP = Constants.shooterP;
        config.slot0.kI = Constants.shooterI;
        config.slot0.kD = Constants.shooterD;
        config.slot0.integralZone = 0;

        motor.configAllSettings(config);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        //motor.set(ControlMode.PercentOutput, shooterSpeed);
        motor.set(ControlMode.Velocity, shooterSpeed);
        System.out.println(motor.getSelectedSensorVelocity());
    }

    @Override
    public void stop() {
        shooterSpeed = 0;
        motor.set(ControlMode.PercentOutput,0);
    }

    public void shoot(double speed, boolean freeze) {
        if (freeze) {
            return;
        }
        if (Math.abs(speed) < Constants.joystickDeadband) {
            shooterSpeed = 0;
            return;
        }
        shooterSpeed = speed * Constants.shooterMaxSpeed;
    }
}