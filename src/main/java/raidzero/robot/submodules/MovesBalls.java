package raidzero.robot.submodules;

public class MovesBalls extends Submodule {

    private static MovesBalls instance = null;

    public static MovesBalls getInstance() {
        if (instance == null) {
            instance = new MovesBalls();
        }
        return instance;
    }

    private MovesBalls() {
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