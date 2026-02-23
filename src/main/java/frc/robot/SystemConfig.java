package frc.robot;
import frc.robot.SystemConfig.Aliases.LoggingLevel;
import frc.robot.SystemConfig.Aliases.Mode;

public final class SystemConfig {
    public static class Aliases {
        public static class Mode {
            public static final int REAL = 0;
            public static final int SIM = 1;
            public static final int REPLAY = 2;
        }
        public static class LoggingLevel {
            public static final int NONE = 0;
            public static final int BASIC = 1;
            public static final int DEFAULT = 2;
            public static final int DEBUG = 3;
        }
    }
    public static final int robotMode = Mode.REAL;
    public static final int logLevel = LoggingLevel.DEFAULT;
}
