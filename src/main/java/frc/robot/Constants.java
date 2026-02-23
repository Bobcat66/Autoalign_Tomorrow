package frc.robot;

public final class Constants {
    public static final class IO {
        public static final double kControllerDeadband = 0.05; // Deadband for controller inputs
        public static final int kDriverControllerPort = 0; // Port for the driver controller
        public static final int kOperatorControllerPort = 1; // Port for the operator controller
    }
    public static final class TeleopDriveK {
        public static final double kMaxLinearSpeed = 5.0; // Maximum linear speed in meters per second
        public static final double kMaxAngularSpeed = Math.PI; // Maximum angular speed in radians per second
    }
}
