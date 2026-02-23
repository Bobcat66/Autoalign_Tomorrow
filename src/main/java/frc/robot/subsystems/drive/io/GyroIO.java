package frc.robot.subsystems.drive.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface GyroIO {
    
    @AutoLog
    public static class GyroIOInputs {
        public GyroIOData data = new GyroIOData(
            false, // connected
            new Rotation2d(0.0), // yaw
            0.0, // yawVelocityRadPerSec
            new Rotation2d(0.0), // pitch
            0.0, // pitchVelocityRadPerSec
            new Rotation2d(0.0), // roll
            0.0 // rollVelocityRadPerSec
        );

        public Rotation3d[] odometryPositions = new Rotation3d[] {};
    }

    public static record GyroIOData(
        boolean connected,
        Rotation2d yaw,
        double yawVelocityRadPerSec,
        Rotation2d pitch,
        double pitchVelocityRadPerSec,
        Rotation2d roll,
        double rollVelocityRadPerSec
    ) {}
    
    public abstract void updateInputs(GyroIOInputs inputs);

    // Resets the yaw
    public abstract void resetYaw(Rotation2d rotation);
}
