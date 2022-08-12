// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.robot.constants.*;

public class SwerveDriveBase extends MustangSubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0 *
   SdsModuleConfigurations.MK4I_L1.getDriveReduction() * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
  
  /**
   * The maximum angular velocity of the robot in radians per second.
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
 private final NavX m_navx; 

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private Rotation2d gyroOffset;
  private double frontLeftPrevAngle, frontRightPrevAngle, backLeftPrevAngle, backRightPrevAngle;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public SwerveDriveBase() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    m_navx = new NavX(RobotMap.NAVX_PORT);
    m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo( 
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4iSwerveModuleHelper.GearRatio.L1,
            // This is the ID of the drive motor
            RobotMap.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            RobotMap.FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            RobotMap.FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            RobotMap.FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            RobotMap.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            RobotMap.FRONT_RIGHT_MODULE_STEER_MOTOR,
            RobotMap.FRONT_RIGHT_MODULE_STEER_ENCODER,
            RobotMap.FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            RobotMap.BACK_LEFT_MODULE_DRIVE_MOTOR,
            RobotMap.BACK_LEFT_MODULE_STEER_MOTOR,
            RobotMap.BACK_LEFT_MODULE_STEER_ENCODER,
            RobotMap.BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            RobotMap.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            RobotMap.BACK_RIGHT_MODULE_STEER_MOTOR,
            RobotMap.BACK_RIGHT_MODULE_STEER_ENCODER,
            RobotMap.BACK_RIGHT_MODULE_STEER_OFFSET
    );
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
        SmartDashboard.putString("zeroGyro", "called zeroGyroscope");
        gyroOffset = getGyroscopeRotation(false);
  }

  public Rotation2d getGyroscopeRotation() {
          return getGyroscopeRotation(true);
  }
  public Rotation2d getGyroscopeRotation(boolean offset) {
    
    if (m_navx.isMagnetometerCalibrated()) {
        
     // We will only get valid fused headings if the magnetometer is calibrated
        if (offset) {
                Rotation2d angle = Rotation2d.fromDegrees(-m_navx.getFusedHeading()).minus(gyroOffset);
                
                return angle;  
        }
        return Rotation2d.fromDegrees(-m_navx.getFusedHeading());
    }

    SmartDashboard.putNumber("navX", m_navx.getYawFieldCentric()- 360);
 // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        if (offset) {      
                return Rotation2d.fromDegrees(m_navx.getYawFieldCentric()-360).minus(gyroOffset);
        }    
        return Rotation2d.fromDegrees(m_navx.getYawFieldCentric()-360);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void mustangPeriodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    //Testing purpose
    for(int i = 0; i < 4; i++) {
        SmartDashboard.putNumber("module # " + i + " speed", states[i].speedMetersPerSecond);
        SmartDashboard.putNumber("module # " + i + " angle", states[i].angle.getRadians());
    }
    
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    if (gyroOffset == null && !m_navx.isCalibrating()) {
            zeroGyroscope();
    }

        double frontLeftSpeed = states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;
        double frontRightSpeed = states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;
        double backLeftSpeed = states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;
        double backRightSpeed =  states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE;

        double frontLeftAngle = states[0].angle.getRadians();
        double frontRightAngle = states[1].angle.getRadians();
        double backLeftAngle = states[2].angle.getRadians();
        double backRightAngle = states[3].angle.getRadians();

        if (Math.abs(frontLeftSpeed) <= 0.01 && Math.abs(frontRightSpeed) <= 0.01 && Math.abs(backLeftSpeed) <= 0.01 && Math.abs(backRightSpeed) <= 0.01) {
                frontLeftAngle = frontLeftPrevAngle;
                frontRightAngle = frontRightPrevAngle;
                backLeftAngle = backLeftPrevAngle;
                backRightAngle = backRightPrevAngle;
        } 

        m_frontLeftModule.set(frontLeftSpeed, frontLeftAngle);
        m_frontRightModule.set(frontRightSpeed, frontRightAngle);
        m_backLeftModule.set(backLeftSpeed, backLeftAngle);
        m_backRightModule.set(backRightSpeed, backRightAngle);

        frontLeftPrevAngle = frontLeftAngle;
        frontRightPrevAngle = frontRightAngle;
        backLeftPrevAngle = backLeftAngle;
        backRightPrevAngle = backRightAngle;
   }

@Override
public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return null;
}


@Override
public void debugSubsystem() {
        // TODO Auto-generated method stub
        
}
}