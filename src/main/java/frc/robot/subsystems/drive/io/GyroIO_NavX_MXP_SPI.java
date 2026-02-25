package frc.robot.subsystems.drive.io;

import static frc.robot.subsystems.drive.DriveConstants.kOdometryFrequencyHz;

import java.util.Objects;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.stream.Stream;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.OdometryThread;
import frc.robot.utils.StreamUtils;

public class GyroIO_NavX_MXP_SPI implements GyroIO {
    private final AHRS m_navx = new AHRS(NavXComType.kMXP_SPI,kOdometryFrequencyHz);
    private final ConcurrentLinkedQueue<Double> odometryRollPositionsRadians;
    private final ConcurrentLinkedQueue<Double> odometryPitchPositionsRadians;
    private final ConcurrentLinkedQueue<Double> odometryYawPositionsRadians;

    private double last_timestamp;
    private double last_roll;
    private double last_pitch;

    public GyroIO_NavX_MXP_SPI() {
        m_navx.reset();
        odometryRollPositionsRadians = OdometryThread.getInstance().registerSignal(() -> Units.degreesToRadians(m_navx.getRoll()));
        odometryPitchPositionsRadians = OdometryThread.getInstance().registerSignal(() -> Units.degreesToRadians(m_navx.getPitch()));
        odometryYawPositionsRadians = OdometryThread.getInstance().registerSignal(() -> Units.degreesToRadians(m_navx.getYaw()));
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        double current_timestamp = Timer.getFPGATimestamp();
        double current_roll = m_navx.getRoll();
        double current_pitch = m_navx.getPitch();
        inputs.data = new GyroIOData(
            m_navx.isConnected(),
            new Rotation2d(m_navx.getYaw()),
            Units.degreesToRadians(m_navx.getRate()),
            new Rotation2d(m_navx.getPitch()),
            Units.degreesToRadians((current_pitch - last_pitch)/(current_timestamp - last_timestamp)),
            new Rotation2d(m_navx.getRoll()),
            Units.degreesToRadians((current_roll-last_roll)/(current_timestamp - last_timestamp))
        );

        last_timestamp = current_timestamp;
        last_roll = current_roll;
        last_pitch = current_pitch;

        inputs.odometryPositions = StreamUtils.trizip(
            Stream.generate(odometryRollPositionsRadians::poll)
                .takeWhile(Objects::nonNull)
                .limit(OdometryThread.getInstance().getSampleCount()),
            Stream.generate(odometryPitchPositionsRadians::poll)
                .takeWhile(Objects::nonNull)
                .limit(OdometryThread.getInstance().getSampleCount()),
            Stream.generate(odometryYawPositionsRadians::poll)
                .takeWhile(Objects::nonNull)
                .limit(OdometryThread.getInstance().getSampleCount()),
            (roll,pitch,yaw) -> new Rotation3d(roll,pitch,yaw)
        ).toArray(Rotation3d[]::new);
    }

    @Override
    public void resetYaw(Rotation2d rotation) {
        m_navx.setAngleAdjustment(rotation.getDegrees());
        m_navx.zeroYaw();
    }
}
