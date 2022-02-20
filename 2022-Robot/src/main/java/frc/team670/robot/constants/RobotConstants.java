package frc.team670.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import frc.team670.mustanglib.constants.RobotConstantsBase;
import edu.wpi.first.math.geometry.*;

public class RobotConstants extends RobotConstantsBase {

        public static final double CAMERA_HEIGHT_METERS = 0.75; // TODO: Change this when we get the robot
        public static final double CAMERA_ANGLE_DEGREES = 25;
        public static final double CAMERA_DISTANCE_TO_FRONT = 0.22; // TODO: Change for 2022

        /** the id for the camera that tells you how to shoot the ball */
        public static final String VISION_CAMERA_NAME = "Microsoft_LifeCam_HD-3000";
        public static final double DRIVEBASE_VELOCITY_CONVERSION_FACTOR = RobotConstants.DRIVEBASE_METERS_PER_ROTATION
                        / 60;
        public static final double HDRIVE_VELOCITY_CONVERSION_FACTOR = RobotConstants.HDRIVE_METERS_PER_ROTATION / 60;

        // Robot Dimensions in Inches
        public static final double ROBOT_LENGTH = 29.5, ROBOT_WIDTH = 30.5, DRIVEBASE_TO_GROUND = 2.03;

        public static final double ROBOT_FULL_LENGTH_WITH_BUMPER = 36;

        // Drive Base Gearing
        public static final double DRIVEBASE_GEAR_RATIO = 10.71;

        // Drive Base Gearing
        public static final double HDRIVE_GEAR_RATIO = 10.71;

        /** Drive Base Wheel Diameter in Inches */
        public static final double DRIVE_BASE_WHEEL_DIAMETER = 6;

        /** HDrive Base Wheel Diameter in Inches */
        public static final double HDRIVE_WHEEL_DIAMETER = 4; // TODO: Check with mech

        /** Inches per rotation of the NEO motors on the drivebase */
        public static final double DRIVEBASE_INCHES_PER_ROTATION = 1 / DRIVEBASE_GEAR_RATIO * DRIVE_BASE_WHEEL_DIAMETER
                        * Math.PI;

        /** The number of ticks per inch of wheel travel */
        public static final int DIO_TICKS_PER_INCH = (int) (DIO_TICKS_PER_ROTATION
                        / (Math.PI * DRIVE_BASE_WHEEL_DIAMETER));

        /** The number of meters per roatation of a drivebase wheel */
        public static final double DRIVEBASE_METERS_PER_ROTATION = (1 / DRIVEBASE_GEAR_RATIO)
                        * DRIVE_BASE_WHEEL_DIAMETER
                        * Math.PI * 0.0254;

        /** The number of meters per roatation of a drivebase wheel */
        public static final double HDRIVE_METERS_PER_ROTATION = (1 / HDRIVE_GEAR_RATIO) * HDRIVE_WHEEL_DIAMETER
                        * Math.PI * 0.0254;

        // Talon PID Constants
        public static final int kTimeoutMs = 0;
        public static final double leftKsVolts = 0.14709;
        public static final double leftKvVoltSecondsPerMeter = 2.7426;
        public static final double leftKaVoltSecondsSquaredPerMeter = 0.35449;
        public static final double rightKsVolts = leftKsVolts;
        public static final double rightKvVoltSecondsPerMeter = leftKvVoltSecondsPerMeter;
        public static final double rightKaVoltSecondsSquaredPerMeter = leftKaVoltSecondsSquaredPerMeter;

        // "WHEEL_BASE" is really track width
        public static final double kTrackwidthMeters = 0.7108;

        // VISION Constants

        public static final int VISION_ERROR_CODE = -99999;
        public static final double kHorizontalFOV_DEGREES = 61;
        public static final double kVerticalFOV_DEGREES = 34.3;
        public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV_DEGREES / 2.0));
        public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV_DEGREES / 2.0));
        public static final double TILT_ANGLE_DEGREES = 43;

        public static final String VISION_CAMERA = "Microsoft_LifeCam_HD-3000";

        // Autonomous Constants

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                        kTrackwidthMeters);
        public static final double leftKPDriveVel = 2.2797;
        public static final double leftKIDriveVel = 0.00;
        public static final double leftKDDriveVel = 0.0;

        public static final double rightKPDriveVel = leftKPDriveVel;
        public static final double rightKIDriveVel = 0.00;
        public static final double rightKDDriveVel = 0.0;

        public static final double kMaxSpeedInchesPerSecond = 6;
        public static final double kMaxAccelerationInchesPerSecondSquared = 6;

        public static final double kMaxSpeedMetersPerSecond = 0.25;// 1; //0.305;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;// 1; //0.305;
        public static final double endVelocityMetersPerSecond = 0;

        public static final double kMaxSpeedMetersPerSecond2 = 0.3;// 1; //0.305;
        public static final double kMaxAccelerationMetersPerSecondSquared2 = 0.3;// 1; //0.305;
        public static final double endVelocityMetersPerSecond2 = 0.2;

        public static final DifferentialDriveKinematicsConstraint kAutoPathConstraints = new DifferentialDriveKinematicsConstraint(
                        kDriveKinematics, kMaxSpeedMetersPerSecond);

        public static final DifferentialDriveKinematicsConstraint kAutoPathConstraintsIntaking = new DifferentialDriveKinematicsConstraint(
                        kDriveKinematics, kMaxSpeedMetersPerSecond);

        // public static final

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = .7;
        public static final boolean kNavXReversed = true;

}