package raidzero.robot.submodules;

import java.util.Arrays;
import java.util.List;

public class SubmoduleManager {
    private static SubmoduleManager instance = null;

    private List<Submodule> submodules;

    private SubmoduleManager() {}

    public static SubmoduleManager getInstance() {
        if (instance == null) {
            instance = new SubmoduleManager();
        }
        return instance;
    }

    public void setSubmodules(Submodule... submodules) {
        this.submodules = Arrays.asList(submodules);
    }

    public void onStart(double timestamp) {
        submodules.forEach(o -> o.onStart(timestamp));
    }

    public void onStop(double timestamp) {
        submodules.forEach(Submodule::stop);
    }

    public void onLoop(double timestamp) {
        submodules.forEach(o -> o.update(timestamp));
        submodules.forEach(Submodule::run);
    }
}