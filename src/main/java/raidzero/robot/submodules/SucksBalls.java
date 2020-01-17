package raidzero.robot.submodules;

public class SucksBalls extends Submodule {

    private static SucksBalls instance = null;

    public static SucksBalls getInstance() {
        if (instance == null) {
            instance = new SucksBalls();
        }
        return instance;
    }

    private SucksBalls() {
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