package raidzero.robot.auto;

import raidzero.robot.auto.sequences.AutoSequence;

public class AutoRunner {

    private AutoSequence selectedSequence;

    public void selectSequence(AutoSequence sequence) {
        selectedSequence = sequence;
    }

    public AutoSequence getSelectedSequence() {
        return selectedSequence;
    }

    public void start() {
        if (selectedSequence != null) {
            System.out.println("[Auto] Starting auto sequence '" + selectedSequence.getName() + "'...");
            selectedSequence.run();
        }
    }

    public void stop() {
        if (selectedSequence != null) {
            System.out.println("[Auto] Stopping auto sequence '" + selectedSequence.getName() + "'...");
            selectedSequence.stop();
        }
    }

    public void onLoop(double timestamp) {
        if (selectedSequence != null) {
            selectedSequence.onLoop(timestamp);
        }
    }
}