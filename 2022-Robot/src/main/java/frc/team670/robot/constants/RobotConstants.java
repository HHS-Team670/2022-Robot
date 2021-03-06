package frc.team670.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import frc.team670.mustanglib.constants.RobotConstantsBase;

public class RobotConstants extends RobotConstantsBase {
        
        public static final double DRIVEBASE_VELOCITY_CONVERSION_FACTOR = 
            RobotConstants.DRIVEBASE_METERS_PER_ROTATION / 60;

        // Drive Base Gearing
        public static final double DRIVEBASE_GEAR_RATIO = 10.71;

        // Drive Wheel Diameter in Inches
        public static final double DRIVE_BASE_WHEEL_DIAMETER = 6;

        // Inches per rotation of the NEO motors on the drivebase
        public static final double DRIVEBASE_INCHES_PER_ROTATION = 
            1 / DRIVEBASE_GEAR_RATIO * DRIVE_BASE_WHEEL_DIAMETER * Math.PI;

        // Number of ticks per inch of wheel travel
        public static final int DIO_TICKS_PER_INCH = 
            (int) (DIO_TICKS_PER_ROTATION / (Math.PI * DRIVE_BASE_WHEEL_DIAMETER));

        // Number of meters per roatation of a drivebase/hdrive wheel
        public static final double DRIVEBASE_METERS_PER_ROTATION = 
            (1 / DRIVEBASE_GEAR_RATIO) * DRIVE_BASE_WHEEL_DIAMETER * Math.PI * 0.0254;

        // Talon PID Constants
        public static final int kTimeoutMs = 0;
        public static final double leftKsVolts = 0.4; //0.20806; //0.4; 
        public static final double leftKvVoltSecondsPerMeter = 2.1; //1.3667; //2.7378; 
        public static final double leftKaVoltSecondsSquaredPerMeter = 0.15; //0.21286; //0.5584; //0.333; 
        public static final double rightKsVolts = leftKsVolts;
        public static final double rightKvVoltSecondsPerMeter = leftKvVoltSecondsPerMeter;
        public static final double rightKaVoltSecondsSquaredPerMeter = leftKaVoltSecondsSquaredPerMeter;

        public static final double kTrackwidthMeters = 0.702;

        // Vision Constants
        public static final double kHorizontalFOV_DEGREES = 61;
        public static final double kVerticalFOV_DEGREES = 34.3;

        public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV_DEGREES / 2.0));
        public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV_DEGREES / 2.0));

        public static final double TILT_ANGLE_DEGREES = 43;
        public static final int VISION_ERROR_CODE = -99999;
        public static final String VISION_CAMERA_NAME = "Microsoft_LifeCam_HD-3000";
        public static final String DRIVER_CAMERA_NAME = "HD_USB_Camera (1)";
        public static final double CAMERA_HEIGHT_METERS = 0.75;
        public static final double CAMERA_ANGLE_DEGREES = 25;

        public static final int LED_START_INDEX = 0;
        public static final int LED_END_INDEX = 70;

        // Autonomous Constants
        public static final DifferentialDriveKinematics kDriveKinematics = 
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double leftKPDriveVel = 2;
        public static final double leftKIDriveVel = 0;
        public static final double leftKDDriveVel = 0;

        public static final double rightKPDriveVel = leftKPDriveVel;
        public static final double rightKIDriveVel = leftKIDriveVel;
        public static final double rightKDDriveVel = leftKDDriveVel;

        public static final double kMaxSpeedInchesPerSecond = 6;
        public static final double kMaxAccelerationInchesPerSecondSquared = 6;

        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double endVelocityMetersPerSecond = 0;

        public static final double kMaxSpeedMetersPerSecond2 = 0.3;
        public static final double kMaxAccelerationMetersPerSecondSquared2 = 0.3;
        public static final double endVelocityMetersPerSecond2 = 0.2;

        public static final DifferentialDriveKinematicsConstraint kAutoPathConstraints = 
            new DifferentialDriveKinematicsConstraint(kDriveKinematics, kMaxSpeedMetersPerSecond);

        public static final DifferentialDriveKinematicsConstraint kAutoPathConstraintsIntaking = 
            new DifferentialDriveKinematicsConstraint(kDriveKinematics, kMaxSpeedMetersPerSecond);

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = .7;
        public static final boolean kNavXReversed = true;
        
        // Robot Dimensions in Inches
        public static final double ROBOT_LENGTH = 29.5,
                                   ROBOT_WIDTH = 30.5,
                                   DRIVEBASE_TO_GROUND = 2.03,
                                   ROBOT_FULL_LENGTH_WITH_BUMPER = 36;
        
}