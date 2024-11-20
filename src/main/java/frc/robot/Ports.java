package frc.robot;

public class Ports {
	
	public class CANDevices {
		public class Talons {
			public static final int SWERVE_FRONT_RIGHT_AZIMUTH= 1; 
			public static final int SWERVE_FRONT_RIGHT_DRIVE   = 0; 
			public static final int SWERVE_FRONT_LEFT_AZIMUTH = 16;
			public static final int SWERVE_FRONT_LEFT_DRIVE    = 17;
			public static final int SWERVE_REAR_LEFT_AZIMUTH  = 15;
			public static final int SWERVE_REAR_LEFT_DRIVE     = 14;
			public static final int SWERVE_REAR_RIGHT_AZIMUTH = 5;
			public static final int SWERVE_REAR_RIGHT_DRIVE    = 4;
			public static final int ARM_LEADER  = 18;
			public static final int ARM_FOLLOWER  = 3;
			public static final int INTAKE = 7;
			public static final int SHOOTER = 6;
			public static final int CLIMBER_LEADER = 2;
			public static final int CLIMBER_FOLLOWER = 19;
		}

		public class Encoders {
			public static final int SWERVE_FRONT_RIGHT = 20;
			public static final int SWERVE_FRONT_LEFT = 21;
			public static final int SWERVE_REAR_LEFT = 22;
			public static final int SWERVE_REAR_RIGHT = 23;
			public static final int ARM = 24;
		}

		public static final int CANDLE = 25;
		public static final int PIGEON = 26;
	}

	public class DigitalDevices {
		public static final int INTAKE_BANNER = 8;
	}

	public class AnalogDevices {}

}
	