package frc.robot.subsystems.drive;


import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/* 
 * This is the DriveConstants file
 * 
 * This file contains constant values needed to set up and configure the drivetrain properly
 */
public final class DriveConstants {

    public static final int kOdometryFrequencyHz = 250;
    public static final int kGyroPort = 9;
    public static final double kMass = 50.0; // kg
    public static final double kMOI = 1.0; // kg*m^2

    public static final double kWheelBase = 0.558; //Meters
    public static final double kTrackWidth = 0.558; //Meters
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
        new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
        new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
        new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0)
    };
    
    public static class ModuleK {
        
        public static class Common {

            public static final double kMaxModuleSpeed = 14.0; // Maximum attainable module speed (Meters per second)
            public static final double kWheelRadius = Units.inchesToMeters(4); //Meters
            public static final double kWheelCOF = 1.0; //Coefficient of friction

            public static class DriveMotorK {
                public static final DCMotor kMotorModel = DCMotor.getNEO(1); // Motor model for the drive motor
                public static final int kCurrentLimit = 60;
                public static final boolean kVoltageCompensation = true;
                public static final double kNominalVoltage = 12.0;
                public static final double kGearReduction = 6.75;

                //PID constants
                public static final double kP = 0.035;
                public static final double kI = 0.000;
                public static final double kD = 0.0012;

                //Feedforward constants
                public static final double kS = 0.0;
                public static final double kV = 2.78;

            }

            public static class SteerMotorK {
                public static final DCMotor kMotorModel = DCMotor.getNEO(1); // Motor model for the steer motor
                public static final int kCurrentLimit = 60;
                public static final boolean kVoltageCompensation = true;
                public static final double kNominalVoltage = 12;
                public static final double kGearRatio = 12.8;

                // PID constants
                public static final double kP = 0.75;
                public static final double kI = 0.0;
                public static final double kD = 0.0001;

                //Feedforward constants
                public static final double kS = 0.0;
                public static final double kV = 2.78;
                public static final double kA = 0.0;
            }
        }
        public static enum ModuleConfig {
    
            FrontLeft(2,1,21,-0.441162109375 +0.5),
            FrontRight(4,3,22,-0.3984375 +0.5),
            RearLeft(8,7,23,-0.525146484375),
            RearRight(6,5,24,-0.931396484375);
    
            public final int DrivePort;
            public final int SteerPort;
            public final int EncoderPort;
            public final double EncoderOffsetRots;
    
            private ModuleConfig(int DrivePort, int SteerPort,int EncoderPort,double EncoderOffsetRots) {
                this.DrivePort = DrivePort;
                this.SteerPort = SteerPort;
                this.EncoderPort = EncoderPort;
                this.EncoderOffsetRots = EncoderOffsetRots;
            }
        }
    }

    public static class AutonK {

        public static class TranslationK {
            public static final double kP = 1.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static class RotationK {
            public static final double kP = 1.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }

        public static final PathConstraints kPathConstraints = new PathConstraints(3, 3, Units.degreesToRadians(360), Units.degreesToRadians(360));
        public static final double kPathTolerance = 0.1; // Meters
    }
}
