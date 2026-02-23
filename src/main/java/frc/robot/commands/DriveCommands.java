package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public final class DriveCommands {

    private DriveCommands () {}

    public static Command driveFieldRelative(
        DoubleSupplier xSpeedSupplier, 
        DoubleSupplier ySpeedSupplier, 
        DoubleSupplier omegaSupplier,
        Drive drive
    ) {
        return drive.run(() -> {
            var speeds = new ChassisSpeeds(
                xSpeedSupplier.getAsDouble(), 
                ySpeedSupplier.getAsDouble(), 
                omegaSupplier.getAsDouble()
            );
            drive.applyFieldSpeeds(speeds);
        });
    }

    public static Command driveRobotRelative(
        DoubleSupplier xSpeedSupplier, 
        DoubleSupplier ySpeedSupplier, 
        DoubleSupplier omegaSupplier,
        Drive drive
    ) {
        return drive.run(() -> {
            var speeds = new ChassisSpeeds(
                xSpeedSupplier.getAsDouble(), 
                ySpeedSupplier.getAsDouble(), 
                omegaSupplier.getAsDouble()
            );
            drive.applyRobotSpeeds(speeds);
        });
    }
}
