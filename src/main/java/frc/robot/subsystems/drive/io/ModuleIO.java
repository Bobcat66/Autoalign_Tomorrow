package frc.robot.subsystems.drive.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;


public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs{
        public ModuleIOData data = new ModuleIOData(
            0.0, // driveRotations
            0.0, // driveVelocityRPM
            0.0, // driveAppliedVolts
            0.0, // driveCurrentAmps
            new Rotation2d(0.0), // steerAbsoluteHeading
            new Rotation2d(0.0), // steerHeading
            0.0, // steerVelocityRPM
            0.0, // steerAppliedVolts
            0.0 // steerCurrentAmps
        );
        
        public double[] odometryDriveRotations = new double[] {};
        public Rotation2d[] odometrySteerHeadings = new Rotation2d[] {};
    }

    public static record ModuleIOData(
        double driveRotations,
        double driveVelocityRPM,
        double driveAppliedVolts,
        double driveCurrentAmps,
        Rotation2d steerAbsoluteHeading,
        Rotation2d steerHeading,
        double steerVelocityRPM,
        double steerAppliedVolts,
        double steerCurrentAmps
    ) {}

    public abstract String getName();

    public abstract void updateInputs(ModuleIOInputs inputs);

    public abstract void rezeroRelativeEncoder(); // Rezeros the relative encoder based on the absolute encoder's reading

    public abstract void setDriveVolts(double volts);

    public abstract void setSteerVolts(double volts);

    public abstract void setDriveVelocity(double velocityRPM, double FFVolts);

    public abstract void setSteerHeading(double headingRotations);
}