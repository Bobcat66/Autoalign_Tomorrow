package frc.robot.subsystems.drive.io;

import java.util.concurrent.ConcurrentLinkedQueue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.OdometryThread;
import frc.robot.subsystems.drive.io.GyroIO.GyroIOData;
import frc.robot.subsystems.drive.io.GyroIO.GyroIOInputs;
import frc.robot.utils.StreamUtils;

import static frc.robot.subsystems.drive.DriveConstants.kGyroPort;
import static frc.robot.subsystems.drive.DriveConstants.kOdometryFrequencyHz;

public class GyroIO_Pigeon2 implements GyroIO {
    private final Pigeon2 m_pigeon = new Pigeon2(kGyroPort);
    private final StatusSignal<Angle> yaw = m_pigeon.getYaw();
    private final StatusSignal<Angle> pitch = m_pigeon.getPitch();
    private final StatusSignal<Angle> roll = m_pigeon.getRoll();
    private final StatusSignal<AngularVelocity> yawVelocity = m_pigeon.getAngularVelocityZWorld();
    private final StatusSignal<AngularVelocity> pitchVelocity = m_pigeon.getAngularVelocityXWorld();
    private final StatusSignal<AngularVelocity> rollVelocity = m_pigeon.getAngularVelocityYWorld();
    private final ConcurrentLinkedQueue<Double> odometryRollPositionsRadians;
    private final ConcurrentLinkedQueue<Double> odometryPitchPositionsRadians;
    private final ConcurrentLinkedQueue<Double> odometryYawPositionsRadians;

    public GyroIO_Pigeon2() {
        m_pigeon.getConfigurator().apply(new Pigeon2Configuration());
        m_pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(kOdometryFrequencyHz);
        roll.setUpdateFrequency(kOdometryFrequencyHz);
        pitch.setUpdateFrequency(kOdometryFrequencyHz);
        BaseStatusSignal.setUpdateFrequencyForAll(50, yawVelocity, pitchVelocity, rollVelocity);
        m_pigeon.optimizeBusUtilization();
        odometryRollPositionsRadians = OdometryThread.getInstance().registerSignal(() -> Units.degreesToRadians(roll.getValueAsDouble()));
        odometryPitchPositionsRadians = OdometryThread.getInstance().registerSignal(() -> Units.degreesToRadians(pitch.getValueAsDouble()));
        odometryYawPositionsRadians = OdometryThread.getInstance().registerSignal(() -> Units.degreesToRadians(yaw.getValueAsDouble()));
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.data = new GyroIOData(
            m_pigeon.isConnected(),
            new Rotation2d(yaw.getValue()),
            yawVelocity.getValue().in(RadiansPerSecond),
            new Rotation2d(pitch.getValue()),
            pitchVelocity.getValue().in(RadiansPerSecond),
            new Rotation2d(roll.getValue()),
            rollVelocity.getValue().in(RadiansPerSecond)
        );

        inputs.odometryPositions = StreamUtils.trizip(
            odometryRollPositionsRadians.stream(),
            odometryPitchPositionsRadians.stream(),
            odometryYawPositionsRadians.stream(),
            (roll,pitch,yaw) -> new Rotation3d(roll,pitch,yaw)
        ).toArray(Rotation3d[]::new);
    }

    @Override
    public void resetYaw(Rotation2d rotation) {
        m_pigeon.setYaw(rotation.getDegrees());
    }

}
