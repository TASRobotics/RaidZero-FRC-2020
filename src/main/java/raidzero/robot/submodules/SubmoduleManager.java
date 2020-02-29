package raidzero.robot.submodules;

import java.util.Arrays;
import java.util.List;

/**
 * Class that can call methods that apply to all submodules.
 */
public class SubmoduleManager {

    private static SubmoduleManager instance = null;

    public static SubmoduleManager getInstance() {
        if (instance == null) {
            instance = new SubmoduleManager();
        }
        return instance;
    }

    private SubmoduleManager() {
    }

    private List<Submodule> submodules;

    /**
     * Sets the list of submodules.
     * 
     * @param submodules varargs list of submodules
     */
    public void setSubmodules(Submodule... submodules) {
        this.submodules = Arrays.asList(submodules);
    }

    /**
     * Calls the {@link Submodule#onInit()} method for all submodules.
     * 
     * @param timestamp
     */
    public void onInit() {
        submodules.forEach(o -> o.onInit());
    }

    /**
     * Calls the {@link Submodule#onStart(double)} method for all submodules.
     * 
     * @param timestamp
     */
    public void onStart(double timestamp) {
        submodules.forEach(o -> o.onStart(timestamp));
    }

    /**
     * Calls the {@link Submodule#stop()} method for all submodules.
     * 
     * @param timestamp
     */
    public void onStop(double timestamp) {
        submodules.forEach(Submodule::stop);
    }

    /**
     * Calls the {@link Submodule#update(double)} and {@link Submodule#run()}
     * method for all submodules.
     * 
     * @param timestamp
     */
    public void onLoop(double timestamp) {
        submodules.forEach(o -> o.update(timestamp));
        submodules.forEach(Submodule::run);
    }
}
