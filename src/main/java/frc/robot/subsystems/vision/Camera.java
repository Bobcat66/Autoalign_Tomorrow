package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import frc.robot.subsystems.vision.io.CameraIO;
import frc.robot.utils.VirtualSubsystem;

public class Camera extends VirtualSubsystem {
    private final CameraIO io;
    private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();
    private List<CameraIO.AprilTagPoseObservation> poseObservationCache = new ArrayList<>();

    public Camera(CameraIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    CameraIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public List<CameraIO.AprilTagPoseObservation> getPoseObservations() {
        return Collections.unmodifiableList(poseObservationCache);
    }
}
