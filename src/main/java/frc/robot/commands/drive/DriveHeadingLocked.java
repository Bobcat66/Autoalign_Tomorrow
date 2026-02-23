package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.AutonK.RotationK;
import frc.robot.utils.GeometryUtils;

// Drives holonomic while aligning the robot with a pose. It controls rotation with PID while allowing the user full translational control
public class DriveHeadingLocked extends Command {
    private final Pose2d targetPose;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final PIDController rotationController = new PIDController(RotationK.kP,RotationK.kI,RotationK.kD);
    private final Drive m_drive;
    public DriveHeadingLocked(Pose2d targetPose, DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier, Drive drive) {
        this.targetPose = targetPose;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        m_drive = drive;
    }

    @Override
    public void initialize() {
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        var speeds = new ChassisSpeeds(
            xSpeedSupplier.getAsDouble(),
            ySpeedSupplier.getAsDouble(),
            rotationController.calculate(GeometryUtils.angleToPose(m_drive.getPose().toPose2d(),targetPose).getRadians(),0)
        );
        m_drive.applyFieldSpeeds(speeds);
    }
}
