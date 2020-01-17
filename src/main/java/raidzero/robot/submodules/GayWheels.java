package raidzero.robot.submodules;

public class GayWheels extends Submodule {

    private static GayWheels instance = null;

    public static GayWheels getInstance() {
        if (instance == null) {
            instance = new GayWheels();
        }
        return instance;
    }

    private GayWheels() {
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