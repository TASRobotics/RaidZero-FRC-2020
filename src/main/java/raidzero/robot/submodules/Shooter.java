package raidzero.robot.submodules;

import raidzero.robot.wrappers.LazyTalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import raidzero.robot.Constants;
import raidzero.robot.Constants.ShooterConstants;

public class Shooter extends Submodule {

    private static LazyTalonFX shooterMotor;
    private static Shooter instance = null;

    private double shooterSpeed = 0.0;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        shooterMotor = new LazyTalonFX(Constants.shooterMotorId);
        shooterMotor.setInverted(ShooterConstants.inversion);
        shooterMotor.setNeutralMode(NeutralMode.Coast);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        config.slot0.kF = ShooterConstants.kF;
        config.slot0.kP = ShooterConstants.kP;
        config.slot0.kI = ShooterConstants.kI;
        config.slot0.kD = ShooterConstants.kD;
        config.slot0.integralZone = ShooterConstants.kIntegralZone;

        shooterMotor.configAllSettings(config);
    }

    @Override
    public void run() {
        //motor.set(ControlMode.PercentOutput, shooterSpeed);
        shooterMotor.set(ControlMode.Velocity, shooterSpeed);
    }

    @Override
    public void stop() {
        shooterSpeed = 0.0;
        shooterMotor.set(ControlMode.PercentOutput, 0);
    }

    public void shoot(double speed, boolean freeze) {
        if (freeze) {
            return;
        }
        shooterSpeed = speed * ShooterConstants.maxSpeed;
    }
}