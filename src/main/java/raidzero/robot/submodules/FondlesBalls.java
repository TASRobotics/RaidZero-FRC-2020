package raidzero.robot.submodules;

public class FondlesBalls extends Submodule {

    private static FondlesBalls instance = null;

    public static FondlesBalls getInstance() {
        if (instance == null) {
            instance = new FondlesBalls();
        }
        return instance;
    }

    private FondlesBalls() {
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