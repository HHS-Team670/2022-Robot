// COPIED FROM 2020

package frc.team670.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import frc.team670.mustanglib.constants.RobotConstantsBase;
import edu.wpi.first.math.geometry.*;

public class RobotConstants extends RobotConstantsBase {
        public static final double CAMERA_HEIGHT_METERS = 1; // TODO: Change this when we get the robot
        public static final double CAMERA_ANGLE_DEGREES = 45;
        public static final Pose2d cameraOffset = new Pose2d();

    /** the id for the camera that tells you how to shoot the ball */
    public static final String TURRET_CAMERA_NAME = "Microsoft_LifeCam_HD-3000";
    public static final double SPARK_MAX_VELOCITY_CONVERSION_FACTOR = RobotConstants.DRIVEBASE_METERS_PER_ROTATION / 60;

    // Robot Dimensions in Inches
    public static final double ROBOT_LENGTH = 29.5, ROBOT_WIDTH = 30.5, DRIVEBASE_TO_GROUND = 2.03;

    public static final double ROBOT_FULL_LENGTH_WITH_BUMPER = 36;

    // Drive Base Gearing
    public static final double DRIVEBASE_GEAR_RATIO = 8.45; // 8.45 if low gear, 10.71 if high gear. TODO check which
                                                            // one it is
    /** Drive Base Wheel Diameter in Inches */
    public static final double DRIVE_BASE_WHEEL_DIAMETER = 6;

    /** Inches per rotation of the NEO motors on the drivebase */
    public static final double DRIVEBASE_INCHES_PER_ROTATION = 1 / DRIVEBASE_GEAR_RATIO * DRIVE_BASE_WHEEL_DIAMETER
            * Math.PI;

    /** The number of ticks per inch of wheel travel */
    public static final int DIO_TICKS_PER_INCH = (int) (DIO_TICKS_PER_ROTATION / (Math.PI * DRIVE_BASE_WHEEL_DIAMETER));

    /** The number of meters per roatation of a drivebase wheel */
    public static final double DRIVEBASE_METERS_PER_ROTATION = (1 / DRIVEBASE_GEAR_RATIO) * DRIVE_BASE_WHEEL_DIAMETER
            * Math.PI * 0.0254;

    // Talon PID Constants
    public static final int kTimeoutMs = 0;
    public static final double leftKsVolts = 0.246;
    public static final double leftKvVoltSecondsPerMeter = 2.1;
    public static final double leftKaVoltSecondsSquaredPerMeter = 0.2;
    public static final double rightKsVolts = 0.12;
    public static final double rightKvVoltSecondsPerMeter = 2.1;
    public static final double rightKaVoltSecondsSquaredPerMeter = 0.1;

    // "WHEEL_BASE" is really track width
    public static final double kTrackwidthMeters = 0.702;

    // VISION Constants

    public static final int VISION_ERROR_CODE = -99999;
    public static final double kHorizontalFOV = 61; // degrees
    public static final double kVerticalFOV = 34.3; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double TILT_ANGLE = 13; // degrees
    public static final double CAMERA_HEIGHT = 0.9525; // meters

    public static final String VISION_CAMERA = "Microsoft_LifeCam_HD-3000";

    // Autonomous Constants

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);
    public static final double leftKPDriveVel = 6;
    public static final double leftKIDriveVel = 0.00;
    public static final double leftKDDriveVel = 0.0;

    public static final double rightKPDriveVel = 2.4;
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

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = .7;
    public static final boolean kNavXReversed = true;

}