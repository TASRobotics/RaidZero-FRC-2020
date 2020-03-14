package raidzero.lib.music;

import java.io.File;
import java.io.FilenameFilter;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class MusicPlayer {

    private static enum State {
        LOADING, PLAYING, PAUSED, STOPPED
    }

    private static final double LOAD_PLAY_WAIT = 0.5; // in seconds

    private List<TalonFX> instruments;
    private Orchestra orchestra;

    private String[] songs;

    private SendableChooser<String> chooser;
    private NetworkTableEntry pausePlayButton;
    private NetworkTableEntry stopButton;
    private NetworkTableEntry playerState;

    private State state = State.STOPPED;

    private Timer timer = new Timer();

    public MusicPlayer(List<TalonFX> instruments) {
        this.instruments = instruments;
        orchestra = new Orchestra(instruments);

        scanSongs();

        chooser = new SendableChooser<>();
        for (String filename : songs) {
            chooser.addOption(filename.substring(0, filename.length() - 5), filename);
        }

        Shuffleboard.getTab("Music")
            .add("Song Selection", chooser)
            .withSize(3, 1)
            .withPosition(0, 0);
        pausePlayButton = Shuffleboard.getTab("Music")
            .add("Play | Pause", false)
            .withSize(1, 1)
            .withPosition(3, 0)
            .getEntry();
        stopButton = Shuffleboard.getTab("Music")
            .add("Stop", false)
            .withSize(1, 1)
            .withPosition(4, 0)
            .getEntry();
        playerState = Shuffleboard.getTab("Music")
            .add("State", State.STOPPED.toString())
            .withSize(2, 1)
            .withPosition(0, 1)
            .getEntry();
    }

    private void scanSongs() {
        File directory = Filesystem.getDeployDirectory();
        FilenameFilter filter = new FilenameFilter() {
            @Override
            public boolean accept(File f, String name) {
                return name.endsWith(".chrp");
            }
        };
        songs = directory.list(filter);
    }

    public boolean loadMusicAndPlay() {
        String selectedSong = chooser.getSelected();
        if (selectedSong == null) {
            return false;
        }
        state = State.LOADING;
        orchestra.loadMusic(selectedSong);
        timer.reset();
        timer.start();
        return true;
    }

    public void update() {
        playerState.setString(state.toString());
        switch (state) {
            case STOPPED:
                if (pausePlayButton.getBoolean(false) && !orchestra.isPlaying() && !loadMusicAndPlay()) {
                    pausePlayButton.setBoolean(false);
                }
                break;
            case LOADING:
                if (timer.hasElapsed(LOAD_PLAY_WAIT) && !orchestra.isPlaying()) {
                    orchestra.play();
                    state = State.PLAYING;
                }
                break;
            case PLAYING:
                if (!pausePlayButton.getBoolean(false) && orchestra.isPlaying()) {
                    orchestra.pause();
                    state = State.PAUSED;
                } else if (stopButton.getBoolean(false)) {
                    orchestra.stop();
                    stopButton.setBoolean(false);
                    pausePlayButton.setBoolean(false);
                    state = State.STOPPED;
                }
                break;
            case PAUSED:
                if (pausePlayButton.getBoolean(false) && !orchestra.isPlaying()) {
                    orchestra.play();
                    state = State.PLAYING;
                } else if (stopButton.getBoolean(false)) {
                    orchestra.stop();
                    stopButton.setBoolean(false);
                    state = State.STOPPED;
                }
                break;
        }
    }
}