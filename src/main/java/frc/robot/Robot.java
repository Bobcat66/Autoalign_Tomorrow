// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.IO.kControllerDeadband;
import static frc.robot.Constants.IO.kDriverControllerPort;
import static frc.robot.Constants.TeleopDriveK.kMaxAngularSpeed;
import static frc.robot.Constants.TeleopDriveK.kMaxLinearSpeed;
import frc.robot.SystemConfig.Aliases.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.ModuleK.ModuleConfig;
import frc.robot.subsystems.drive.io.GyroIO_NavX_MXP_SPI;
import frc.robot.subsystems.drive.io.GyroIO_Pigeon2;
import frc.robot.subsystems.drive.io.ModuleIOHardware;
import frc.robot.subsystems.drive.Module;
import frc.robot.utils.VirtualSubsystem;

public class Robot extends LoggedRobot {

    private final Drive m_drive;
    private final CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);

    @SuppressWarnings("unused")
    public Robot() {

        // Configure AdvantageKit. This must be done BEFORE any other instatiation
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
            case 1 -> Logger.recordMetadata("GitDirty", "Uncomitted changes");
            default -> Logger.recordMetadata("GitDirty", "Unknown");
        }

        // Set up data receivers & replay source
        if (SystemConfig.robotMode == Mode.REAL){
            // Running on a real robot, log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else if (SystemConfig.robotMode == Mode.SIM) {
            // Running a physics simulator, log to NT
            Logger.addDataReceiver(new NT4Publisher());
        } else if (SystemConfig.robotMode == Mode.REPLAY) {
            // Replaying a log, set up replay source
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }
        // Start AdvantageKit logger
        Logger.start();

        // Subsystem Instantiation

        // TODO: Add sim support
        m_drive = new Drive(
            new GyroIO_NavX_MXP_SPI(),
            new Module(new ModuleIOHardware(ModuleConfig.FrontLeft)),
            new Module(new ModuleIOHardware(ModuleConfig.FrontRight)),
            new Module(new ModuleIOHardware(ModuleConfig.RearLeft)),
            new Module(new ModuleIOHardware(ModuleConfig.RearRight))
        );

        m_drive.setDefaultCommand(
            DriveCommands.driveFieldRelative(
                () -> MathUtil.applyDeadband(m_driverController.getLeftX(), kControllerDeadband) * kMaxLinearSpeed, 
                () -> MathUtil.applyDeadband(m_driverController.getLeftY(), kControllerDeadband) * kMaxLinearSpeed,
                () -> MathUtil.applyDeadband(m_driverController.getRightX(), kControllerDeadband) * kMaxAngularSpeed, 
                m_drive
            )
        );
    }

    @Override
    public void robotPeriodic() {
        VirtualSubsystem.periodicAll();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
