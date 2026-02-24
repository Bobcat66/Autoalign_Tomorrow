package frc.robot.subsystems.vision.io;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.proto.PhotonTrackedTargetProto;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;

// Generic interface for AprilTag Localization Source IO layers
public interface CameraIO {

    @AutoLog
    public static class CameraIOInputs {
        public CameraIOData data = new CameraIOData(
            false, // connected
            0,
            0.0,
            0.0
        );
    }

    public static record CameraIOData(
        boolean connected,
        int numTags,
        double fps,
        double latencyMs
    ) {}

    public static record AprilTagPoseObservation(
        double timestampSeconds,
        Pose3d pose,
        List<PhotonTrackedTarget> targetsUsed,
        PhotonPoseEstimator.PoseStrategy strategy,
        Vector<N4> stddevs
    ) {}

    public static record VisionStdDevs(
        double transMultiTagStdDev,
        double rotMultiTagStdDev,
        double transSingleTagStdDev,
        double rotSingleTagStdDev
    ) {}

    @SuppressWarnings("unchecked")
    public static List<Vector<N4>> getSDVectors(VisionStdDevs StdDevs) {
        var multitagStdDevs = VecBuilder.fill(
            StdDevs.transMultiTagStdDev(),
            StdDevs.transMultiTagStdDev(),
            StdDevs.transMultiTagStdDev(),
            StdDevs.rotMultiTagStdDev()
        );
        var singletagStdDevs = VecBuilder.fill(
            StdDevs.transSingleTagStdDev(),
            StdDevs.transSingleTagStdDev(),
            StdDevs.transSingleTagStdDev(),
            StdDevs.rotSingleTagStdDev()
        );
        ArrayList<Vector<N4>> out = new ArrayList<>();
        out.add(multitagStdDevs);
        out.add(singletagStdDevs);
        return out;
    }

    public abstract void updateInputs(CameraIOInputs inputs);

    public abstract String getName();

    // Uses the SQPnP Algorithm to compute multitag pose on the coprocessor, automatically falls back to the Coprocessor's IPPE Square result in the single-tag case
    public abstract List<AprilTagPoseObservation> getAllUnreadPoseObservations();

    // Uses a Constrained PNP solver to compute pose (very resource-intensive)
    public abstract List<AprilTagPoseObservation> getAllUnreadPoseObservations_ConstrainedPNP();

    // TODO: Implement ts
    // Uses a Constrained PNP solver to compute pose from only the best result in the cache
    // public abstract List<AprilTagPoseObservation> getBestConstrainedPnPObservation();

    // Uses a Constrained PNP solver to compute pose from only the latest result in the cache
    public abstract List<AprilTagPoseObservation> getLatestConstrainedPnPObservation();

    // Uses the IPPE Square algorithm to compute tag distance, then solves for 3D pose trigonometrically from the best target
    public abstract List<AprilTagPoseObservation> getAllUnreadPoseObservations_TrigSolvePNP();
}