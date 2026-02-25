package frc.robot.subsystems.drive.io;

import java.util.Objects;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.drive.DriveConstants.ModuleK.Common.DriveMotorK;
import frc.robot.subsystems.drive.DriveConstants.ModuleK.Common.SteerMotorK;
import frc.robot.subsystems.drive.io.ModuleIO.ModuleIOData;
import frc.robot.subsystems.drive.io.ModuleIO.ModuleIOInputs;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.OdometryThread;
import frc.robot.subsystems.drive.DriveConstants.ModuleK;
import frc.robot.subsystems.drive.DriveConstants.ModuleK.ModuleConfig;
import static frc.robot.subsystems.drive.DriveConstants.kOdometryFrequencyHz;

public class ModuleIOHardware implements ModuleIO {

    private final SparkMax m_driveMotor;
    private final SparkClosedLoopController m_driveController;
    private final RelativeEncoder m_driveEncoder;

    private final SparkMax m_steerMotor;
    private final SparkClosedLoopController m_steerController;
    private final RelativeEncoder m_steerEncoder;

    private final ConcurrentLinkedQueue<Double> drivePositionQueue;
    private final ConcurrentLinkedQueue<Double> steerHeadingQueue;

    private final CANcoder m_steerCANcoder;
    private final StatusSignal<Angle> steerAbsolutePosition;

    private final String name;

    public ModuleIOHardware(ModuleConfig config) {

        this.name = config.name();

        // Drive motor acquisition
        m_driveMotor = new SparkMax(config.DrivePort,MotorType.kBrushless);
        m_driveController = m_driveMotor.getClosedLoopController();
        m_driveEncoder = m_driveMotor.getEncoder();

        // Steer motor acquisition
        m_steerMotor = new SparkMax(config.SteerPort,MotorType.kBrushless);
        m_steerController = m_steerMotor.getClosedLoopController();
        m_steerEncoder = m_steerMotor.getEncoder();

        // CANcoder acquisition
        m_steerCANcoder = new CANcoder(config.EncoderPort);

        // CANcoder configuration
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.FutureProofConfigs = false;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        encoderConfig.MagnetSensor.MagnetOffset = -config.EncoderOffsetRots;
        m_steerCANcoder.getConfigurator().apply(encoderConfig);
        steerAbsolutePosition = m_steerCANcoder.getAbsolutePosition();

        // Drive motor configuration
        double driveConversionFactor = 1/DriveMotorK.kGearReduction;
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DriveMotorK.kCurrentLimit)
            .inverted(config.DriveInverted);
        if (DriveMotorK.kVoltageCompensation) driveConfig.voltageCompensation(DriveMotorK.kNominalVoltage);
        driveConfig.encoder
            .positionConversionFactor(driveConversionFactor)
            .velocityConversionFactor(driveConversionFactor);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(DriveMotorK.kP)
            .i(DriveMotorK.kI)
            .d(DriveMotorK.kD);
        driveConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000/kOdometryFrequencyHz))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        m_driveMotor.configure(driveConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_driveEncoder.setPosition(steerAbsolutePosition.getValueAsDouble());

        // Steer motor configuration
        double steerConversionFactor = 1/SteerMotorK.kGearRatio;
        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(SteerMotorK.kCurrentLimit)
            .inverted(config.SteerInverted);
        if (SteerMotorK.kVoltageCompensation) steerConfig.voltageCompensation(SteerMotorK.kNominalVoltage);
        steerConfig.encoder
            .positionConversionFactor(steerConversionFactor)
            .velocityConversionFactor(steerConversionFactor);
        steerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                SteerMotorK.kP,
                SteerMotorK.kI,
                SteerMotorK.kD
            )
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0,1);
        steerConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000/kOdometryFrequencyHz))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        
        m_steerMotor.configure(steerConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_steerEncoder.setPosition(steerAbsolutePosition.getValueAsDouble());
        
        // Registering signals
        OdometryThread.getInstance().registerErrorSignal(m_driveMotor::hasActiveFault);
        OdometryThread.getInstance().registerErrorSignal(m_steerMotor::hasActiveFault);
        drivePositionQueue = OdometryThread.getInstance().registerSignal(m_driveEncoder::getPosition);
        steerHeadingQueue = OdometryThread.getInstance().registerSignal(m_steerEncoder::getPosition);
    }

    @Override
    public String getName(){
        return name;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs){
        int samples = OdometryThread.getInstance().getSampleCount();
        /** Should be called after poll() in the main thread */
        inputs.data = new ModuleIOData(
            m_driveEncoder.getPosition(),
            m_driveEncoder.getVelocity(),
            m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage(),
            m_driveMotor.getOutputCurrent(),
            new Rotation2d(steerAbsolutePosition.getValue()),
            Rotation2d.fromRotations(m_steerEncoder.getPosition()),
            m_steerEncoder.getVelocity(),
            m_steerMotor.getAppliedOutput() * m_steerMotor.getBusVoltage(),
            m_steerMotor.getOutputCurrent()
        );

        inputs.odometryDriveRotations = DoubleStream.generate(drivePositionQueue::poll)
            .takeWhile(Objects::nonNull)
            .limit(samples)
            .toArray();
        inputs.odometrySteerHeadings = Stream.generate(() -> Rotation2d.fromRotations(steerHeadingQueue.poll()))
            .takeWhile(Objects::nonNull)
            .limit(samples)
            .toArray(Rotation2d[]::new);
    }

    @Override
    public void rezeroRelativeEncoder() {
        /** Rezeros the relative encoder based on the absolute encoder's reading */
        m_steerEncoder.setPosition(steerAbsolutePosition.getValueAsDouble());
    }

    @Override
    public void setDriveVolts(double volts){
        m_driveController.setReference(volts,SparkBase.ControlType.kVoltage);
    }

    @Override
    public void setSteerVolts(double volts){
        m_steerController.setReference(volts,SparkBase.ControlType.kVoltage);
    }

    @Override
    public void setDriveVelocity(double velocityRPM,double FFVolts){
        m_driveController.setReference(
            velocityRPM,
            SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            FFVolts,
            SparkClosedLoopController.ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setSteerHeading(double headingRotations){
        m_steerController.setReference(
            headingRotations,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );
    }

}
