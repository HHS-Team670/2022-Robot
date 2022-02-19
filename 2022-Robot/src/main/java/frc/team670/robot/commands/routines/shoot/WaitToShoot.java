package frc.team670.robot.commands.routines.shoot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.subsystems.Shooter;

//only shoots when the robot is within a desired location
public class WaitToShoot extends CommandBase implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private Shooter shooter;
    private Pose2d target;
    private double error;
    private double distanceFromHub;
  
    //error is in meters
    public WaitToShoot(DriveBase driveBase, Shooter shooter, Pose2d targetPose, double errorInMeters) {
      double distanceX = target.getX() - FieldConstants.HUB_X_POSITION_METERS;
      double distanceY = target.getY() - FieldConstants.HUB_Y_POSITION_METERS;
      distanceFromHub = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
      healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
      healthReqs.put(shooter, HealthState.GREEN);

      this.driveBase = driveBase;
      this.target = targetPose;
      this.error = errorInMeters;
      this.shooter = shooter;
    } 

    /**
     * Creates a WaitToShoot, but with added distance because if the robot is
     * moving while shooting, then the calculated RPM for distance will be inaccurate.
     * @param addedDistance Distance, in meters, to be added to the shooter calculations. Negative numbers will reduce the distance
     */
    public WaitToShoot(DriveBase driveBase, Shooter shooter, Pose2d targetPose, double errorInMeters, double addedDistance) {
      this(driveBase, shooter, targetPose, errorInMeters);
      distanceFromHub+= addedDistance;
    }

    @Override
    public void initialize() {
      shooter.setRPMForDistance(distanceFromHub); 
      shooter.run();
    }

    /**
    * Determines whether or not the robot is within the acceptable margin from the target pose.
    * @return true if the current pose is within the error, false otherwise
    */
    @Override
    public boolean isFinished(){
      double distanceX = driveBase.getPose().getX() - target.getX();
      double distanceY = driveBase.getPose().getY() - target.getY();
      //pythagorean theorem
      double distance = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
      
      if (distance < this.error){
        return true;
      }

      else
        return false;
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
      return healthReqs;
    }
}