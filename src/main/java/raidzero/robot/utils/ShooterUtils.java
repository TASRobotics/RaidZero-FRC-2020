package raidzero.robot.utils;

import raidzero.robot.auto.actions.TurnToGoal;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Turret;

public class ShooterUtils {
    private static TurnToGoal turnAction;
    private static Turret turret = Turret.getInstance();
    private static Shooter shooter = Shooter.getInstance();

    public static void init() {
        turnAction = new TurnToGoal();
        turnAction.start();
    }

    public static void turn() {
        if(turnAction.isFinished()) {
            return;
        }
        turnAction.update();
    }

    public static void shoot() {
        shooter.shoot(1, false); //temp
    }

    public static void aim() {
        turn();
        shoot();
    }

    public static void stop() {
        turret.stop();
        shooter.stop();
    }

}