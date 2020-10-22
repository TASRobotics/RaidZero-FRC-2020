package raidzero.robot.submodules;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CameraManager extends Submodule {

    // Usable ports: 554, 5800-5810 (5800 & 5801 are used by Limelight)
    private static final int GSTREAMER_PORT = 5004;
    private static final String GSTREAMER_IP = "10.42.53.69";

    // Preset constants from the Shuffleboard plugin
    // Ref: https://github.com/4639RoboSpartans/gstreamer_shuffleboard
    private static final String NT_GSTREAMER_TABLE = "GStreamer";
    private static final String NT_URL_KEY = "streams";

    // Name: URL
    private static final HashMap<String, String> CAMERAS = new HashMap<>();
    static {
        //CAMERAS.put("test", "test");
        //CAMERAS.put("test2", "test2");
    }

	private static CameraManager instance = null;

	public static CameraManager getInstance() {
		if (instance == null) {
			instance = new CameraManager();
		}
		return instance;
	}

	private CameraManager() {
        registerCameras();
    }
    
    /**
     * Ref: https://github.com/opensight-cv/opensight/blob/master/opsi/modules/videoio/h264.py#L52-L54
     */
    private void registerCameras() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_GSTREAMER_TABLE);
        for (HashMap.Entry<String, String> camera : CAMERAS.entrySet()) {
            table.getSubTable(camera.getKey()).getEntry(NT_URL_KEY).setStringArray(new String[] {
                String.format("rtsp://%s:%d/%s", GSTREAMER_IP, GSTREAMER_PORT, camera.getValue())
            });
        }
    }

	@Override
	public void stop() {
		
	}
}
