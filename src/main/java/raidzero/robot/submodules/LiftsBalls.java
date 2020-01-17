package raidzero.robot.submodules;

public class LiftsBalls extends Submodule {

    private static LiftsBalls instance = null;

    public static LiftsBalls getInstance() {
        if (instance == null) {
            instance = new LiftsBalls();
        }
        return instance;
    }

    private LiftsBalls() {
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
    }

    @Override
    public void stop() {
    }
}