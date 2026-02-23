package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Newtons;
import edu.wpi.first.units.measure.Force;
import frc.robot.subsystems.drive.DriveConstants.ModuleK.Common.DriveMotorK;
import frc.robot.subsystems.drive.DriveConstants.ModuleK.Common.SteerMotorK;
import frc.robot.subsystems.drive.io.*;

import static frc.robot.subsystems.drive.DriveConstants.ModuleK.Common.kWheelRadius;
public class Module {

    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final ModuleIO io;

    private final SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(DriveMotorK.kS, DriveMotorK.kV);
    
    private final DCMotor m_driveGearbox = DriveMotorK.kMotorModel.withReduction(DriveMotorK.kGearReduction);
    private final DCMotor m_steerGearbox = SteerMotorK.kMotorModel.withReduction(SteerMotorK.kGearRatio);

    private final double rotationsToMeters = Math.PI * 2.0 * kWheelRadius; // Conversion factor from rotations to meters

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(ModuleIO io) {
        this.io = io;
    }

    private void buildOdometryPositions() {
        int samples = OdometryThread.getInstance().getSampleCount();
        odometryPositions = new SwerveModulePosition[samples];
        for (int i = 0; i < samples; i++) {
            odometryPositions[i] = new SwerveModulePosition(
                inputs.odometryDriveRotations[i] * rotationsToMeters,
                inputs.odometrySteerHeadings[i]
            );
        }
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.data.driveRotations() * rotationsToMeters, // Convert rotations to meters
            inputs.data.steerHeading() // Use the steer heading directly
        ); // Returns the current position of the module
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.data.driveVelocityRPM() * rotationsToMeters / 60.0, // Convert RPM to m/s
            inputs.data.steerHeading()
        ); // Returns the current state of the module
    }

    public SwerveModulePosition[] getOdometryPositions() {
        if (odometryPositions.length == 0) {
            buildOdometryPositions(); // Ensure positions are built if not already done
        }
        return odometryPositions;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/" + getName(), inputs);
        buildOdometryPositions();
    }

    public String getName() {
        return io.getName();
    }

    public void setTargetState(SwerveModuleState targetState, Force driveForce) {
        targetState.optimize(inputs.data.steerHeading());
        double driveTorqueNm = driveForce.in(Newtons) * kWheelRadius; // Convert force to torque
        double driveVelocityRPM = (targetState.speedMetersPerSecond * 60) / rotationsToMeters; // Convert m/s to RPM
        double FFVolts = ffModel.calculate(driveVelocityRPM) + m_driveGearbox.rOhms * (driveTorqueNm / m_driveGearbox.KtNMPerAmp); // Convert torque to volts
        io.setDriveVelocity(driveVelocityRPM, FFVolts);
        io.setSteerHeading(targetState.angle.getRotations());
    }

    public void setTargetState(SwerveModuleState targetState){
        targetState.optimize(inputs.data.steerHeading());
        double driveVelocityRPM = (targetState.speedMetersPerSecond * 60) / rotationsToMeters; // Convert m/s to RPM
        double FFVolts = ffModel.calculate(targetState.speedMetersPerSecond); // Convert torque to volts
        io.setDriveVelocity(driveVelocityRPM, FFVolts);
        io.setSteerHeading(targetState.angle.getRotations());
    }


}
