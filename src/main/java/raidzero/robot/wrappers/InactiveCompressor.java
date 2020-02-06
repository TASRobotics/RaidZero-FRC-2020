package raidzero.robot.wrappers;

import edu.wpi.first.wpilibj.Compressor;

public class InactiveCompressor extends Compressor {

    private static InactiveCompressor instance = null;
    
    private boolean state = true;

    public static InactiveCompressor getInstance() {
        if (instance == null) {
            instance = new InactiveCompressor();
        }
        return instance;
    }

    public InactiveCompressor() {
        super();
    }

    public void changeState() {
        state = !state;
        if (state) {
            super.start();
        } else {
            super.stop();
        }
    }
}