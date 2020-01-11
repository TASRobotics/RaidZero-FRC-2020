package raidzero.robot.components;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Base {

    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private PigeonIMU pigeon;

    /**
     * Constructs a Base object and sets up the motors and gear shift.
     *
     * @param lLeaderId the ID of the left leader motor
     * @param lFollowerId the ID of the left follower motor
     * @param rLeaderId the ID of the right leader motor
     * @param rFollowerId the ID of the right follower motor
     * @param pigeonId the ID of the pigeon
     */
    public Base(
        int rLeaderId, int rFollowerId, 
        int lLeaderId, int lFollowerId, 
        int pigeonId) {
        rightMotor = initSide(rLeaderId, rFollowerId, false);
        leftMotor = initSide(lLeaderId, lFollowerId, true);
        pigeon = new PigeonIMU(pigeonId);
    }

    /**
     * Constructs and configures the motors for one side of the robot (i.e. one leader and two
     * followers), and returns the leader motor object.
     *
     * @param leaderId the ID of the leader motor
     * @param followerId the ID of the follower motor
     * @param invert whether to invert the leader or not
     * @return the newly constructed leader motor object
     */
    private TalonFX initSide(int leaderId, int followerId, boolean invert) {
        TalonFX leader = new TalonFX(leaderId);
        TalonFX follower = new TalonFX(followerId);

        leader.configFactoryDefault();
        follower.configFactoryDefault();

        leader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor);
        leader.configNeutralDeadband(0.04);

        leader.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);

        follower.follow(leader);

        leader.setInverted(invert);
        follower.setInverted(invert);

        return leader;
    }

    /**
     * Returns the right leader motor.
     *
     * <p>Anything done to this motor will also be followed by the other right motor.
     *
     * @return the right leader motor
     */
    public TalonFX getRightMotor() {
        return rightMotor;
    }

    /**
     * Returns the left leader motor.
     *
     * <p>Anything done to this motor will also be followed by the other left motor.
     *
     * @return the left leader motor
     */
    public TalonFX getLeftMotor() {
        return leftMotor;
    }

    /**
     * Returns the pigeon.
     *
     * @return the pigeon
     */
    public PigeonIMU getPigeon() {
        return pigeon;
    }

    /**
     * Get the yaw value of the pigeon
     */
    public double getYaw() {
        double[] angles = new double[3];
        pigeon.getYawPitchRoll(angles);
        return angles[0];
    }

}
