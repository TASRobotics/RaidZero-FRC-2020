package raidzero.robot.submodules;

import raidzero.robot.wrappers.LazyTalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants;
import raidzero.robot.Constants.ShooterConstants;

public class Shooter extends Submodule {

    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {}

    private LazyTalonFX shooterMotor;
    
    private double outputPercentSpeed = 0.0;

    @Override
    public void onInit() {
        shooterMotor = new LazyTalonFX(ShooterConstants.MOTOR_ID);
        shooterMotor.configFactoryDefault();
        shooterMotor.setNeutralMode(ShooterConstants.NEUTRAL_MODE);
        shooterMotor.setInverted(ShooterConstants.INVERSION);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        config.slot0.kF = ShooterConstants.K_F;
        config.slot0.kP = ShooterConstants.K_P;
        config.slot0.kI = ShooterConstants.K_I;
        config.slot0.kD = ShooterConstants.K_D;
        config.slot0.integralZone = ShooterConstants.K_INTEGRAL_ZONE;

        shooterMotor.configAllSettings(config);
    }

    @Override
    public void onStart(double timestamp) {
        outputPercentSpeed = 0.0;
    }

    @Override
    public void update(double timestamp) {
        SmartDashboard.putNumber("Shooter Vel", shooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter Target", outputPercentSpeed);
        SmartDashboard.putBoolean("Shooter Up2Speed", isUpToSpeed());
    }

    @Override
    public void run() {
        if (Math.abs(outputPercentSpeed) < 0.1) {
            stop();
        } else {
            shooterMotor.set(ControlMode.Velocity, 
                outputPercentSpeed * ShooterConstants.FAKE_MAX_SPEED);
        }        
    }

    @Override
    public void stop() {
        outputPercentSpeed = 0.0;
        shooterMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zero() {
        shooterMotor.getSensorCollection().setIntegratedSensorPosition(0.0, Constants.TIMEOUT_MS);
    }

    /**
     * Fires up the shooter.
     * 
     * @param percentSpeed speed of the shooter in [-1.0, 1.0]
     * @param freeze       whether to disregard the speed and keep the 
     *                     previous speed
     */
    public void shoot(double percentSpeed, boolean freeze) {
        if (freeze) {
            return;
        }
        outputPercentSpeed = percentSpeed;
    }

    /**
     * Returns whether the shooter is up to the setpoint speed.
     * 
     * @return whether the shooter is up to speed
     */
    public boolean isUpToSpeed() {
        return Math.abs(shooterMotor.getClosedLoopError()) < ShooterConstants.ERROR_TOLERANCE;
    }
}