package frc.team670.robot.commands.routines.shoot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;

/**
 * Only calls isFinished() when robot is within the maximum distance (in meters) from the target pose
 * @author EliseVambenepe, Justin Hwang
*/
public class WaitUntilLocationIsReached extends CommandBase implements MustangCommand {

  private Map<MustangSubsystemBase, HealthState> healthReqs;
  private DriveBase driveBase;
  private Pose2d target;
  private double targetRadius; //Maximum acceptable distance from the target pose

  /**
   * Creates a new command
   * @param driveBase Drivebase object
   * @param targetX x-coordinate of target pose (in meters)
   * @param targetY y-coordinate of target pose (in meters)
   * @param targetRadius Maximum acceptable distance from target point (in meters)
   */
  public WaitUntilLocationIsReached(DriveBase driveBase, double targetX, double targetY, double targetRadius) {
    this(driveBase, new Pose2d(targetX, targetY, Rotation2d.fromDegrees(0)), targetRadius);
  }

  /**
   * Creates a new command
   * @param driveBase Drivebase object
   * @param targetPose The target pose (meters)
   * @param targetRadius Maximum acceptable distance from target point (in meters)
   */
  public WaitUntilLocationIsReached(DriveBase driveBase, Pose2d targetPose, double targetRadius) {
    this.driveBase = driveBase;
    this.target = targetPose;
    this.targetRadius = targetRadius;

    healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    healthReqs.put(driveBase, HealthState.GREEN);
  }

  @Override
  public void initialize() {

  }

  /**
  * Determines whether or not the robot is within the acceptable margin from the target pose.
  * @return true if the current pose is within the error, false otherwise
  */
  @Override
  public boolean isFinished(){
    double distanceX = driveBase.getPose().getX() - target.getX();
    double distanceY = driveBase.getPose().getY() - target.getY();
    double distance = Math.sqrt(distanceX*distanceX + distanceY*distanceY);
    
    return (distance < this.targetRadius);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return healthReqs;
  }
}