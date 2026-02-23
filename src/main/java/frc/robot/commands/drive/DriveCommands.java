package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.AutonK;
import frc.robot.subsystems.drive.DriveConstants.AutonK.RotationK;
import frc.robot.utils.GeometryUtils;

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

    private static PIDController rotationController = new PIDController(RotationK.kP,RotationK.kI,RotationK.kD);
    {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }
    public static Command driveAutoAligned(
        DoubleSupplier xSpeedSupplier,
        DoubleSupplier ySpeedSupplier,
        Pose2d targetPose,
        Drive drive
    ) {
        rotationController.reset();
        return drive.run(
            () -> {
                var speeds = new ChassisSpeeds(
                    xSpeedSupplier.getAsDouble(),
                    ySpeedSupplier.getAsDouble(),
                    rotationController.calculate(GeometryUtils.angleToPose(drive.getPose().toPose2d(),targetPose).getRadians(),0)
                );
                drive.applyFieldSpeeds(speeds);
            }
        );
    }
}
