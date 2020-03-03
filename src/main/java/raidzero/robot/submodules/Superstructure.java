package raidzero.robot.submodules;

import java.util.Arrays;

import raidzero.robot.auto.actions.ReusableSeriesAction;
import raidzero.robot.auto.actions.TurnToGoal;
import raidzero.robot.auto.actions.TurnTurretToAngle;
import raidzero.robot.auto.actions.VisionAssistedTargeting;

public class Superstructure extends Submodule {

    private static Superstructure instance = null;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    private Superstructure() {
    }

    private boolean isAiming = false;
    private TurnToGoal aimAction;

    private boolean isAimingAndHood = false;
    private ReusableSeriesAction aimAndHoodAction;

    private boolean isTurretPIDing = false;
    private TurnTurretToAngle turretPIDAction;

    @Override
    public void onStart(double timestamp) {
        aimAction = new TurnToGoal();
        aimAndHoodAction = new ReusableSeriesAction(Arrays.asList(
            new TurnToGoal(), 
            new VisionAssistedTargeting()
        ));
        turretPIDAction = new TurnTurretToAngle(90);
    }

    @Override
    public void update(double timestamp) {
        if (isAiming) {
            aimAction.update();
        }
        if (isAimingAndHood) {
            aimAndHoodAction.update();
        }
        if (isTurretPIDing) {
            turretPIDAction.update();
        }
    }

    @Override
    public void stop() {
        setAiming(false);
        setAimingAndHood(false);
        setTurretPIDing(false);
    }

    /**
     * Sets the aiming status.
     * 
     * @param status the status
     */
    public void setAiming(boolean status) {
        if (status == isAiming) {
            return;
        }
        isAiming = status;
        if (status) {
            // Don't aim if the robot is aiming and shooting already
            if (isAimingAndHood) {
                isAiming = false;
                return;
            }
            aimAction.start();
        } else {
            aimAction.done();
        }
    }

    /**
     * Sets the aiming & shooting status.
     * 
     * @param status the status
     */
    public void setAimingAndHood(boolean status) {
        if (status == isAimingAndHood) {
            return;
        }
        isAimingAndHood = status;
        if (status) {
            aimAndHoodAction.start();
        } else {
            aimAndHoodAction.done();
        }
    }

    public boolean isTurretPIDing() {
        return isTurretPIDing;
    }

    public void setTurretPIDing(boolean status) {
        if (status == isTurretPIDing) {
            return;
        }
        isTurretPIDing = status;
        if (status) {
            if (isAiming || isAimingAndHood) {
                isTurretPIDing = false;
                return;
            }
            turretPIDAction.start();
        } else {
            turretPIDAction.done();
        }
    }
}
