package frc.team670.robot.commands.routines.shoot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private String hubType;
  
    //default to low hub
    public WaitToShoot(DriveBase driveBase, Shooter shooter, Pose2d targetPose, double errorInMeters) {
      this(driveBase, shooter, targetPose, errorInMeters, "lower");
    } 

    //error is in meters
    //see Shooter.setRPMForDistance(double distance, String hubType) for valid hubTypes
    public WaitToShoot(DriveBase driveBase, Shooter shooter, Pose2d targetPose, double errorInMeters, String hubType) {
      this.driveBase = driveBase;
      this.target = targetPose;
      this.error = errorInMeters;
      this.shooter = shooter;
      this.hubType = hubType.toLowerCase();
     
      double distanceX = target.getX() - FieldConstants.HUB_X_POSITION_METERS;
      double distanceY = target.getY() - FieldConstants.HUB_Y_POSITION_METERS;
      distanceFromHub = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
      healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
      healthReqs.put(shooter, HealthState.GREEN);


    } 

    /**
     * Creates a WaitToShoot, but with added distance because if the robot is
     * moving while shooting, then the calculated RPM for distance will be inaccurate.
     * @param addedDistance Distance, in meters, to be added to the shooter calculations. Negative numbers will reduce the distance
     */
    public WaitToShoot(DriveBase driveBase, Shooter shooter, Pose2d targetPose, double errorInMeters, double addedDistance, String hubType) {
      this(driveBase, shooter, targetPose, errorInMeters, hubType);
      distanceFromHub+= addedDistance;
    }

    @Override
    public void initialize() {
      if (hubType.equals("lower")){
        double lowGoalRPM = shooter.getTargetRPMForLowGoalDistance(distanceFromHub);
        shooter.setTargetRPM(lowGoalRPM);
      } else if (hubType.equals("upper")){
        double upperGoalRPM = shooter.getTargetRPMForHighGoalDistance(distanceFromHub);
        shooter.setTargetRPM(upperGoalRPM);
      }
      // shooter.setRPMForDistance(distanceFromHub); 
      SmartDashboard.putNumber("target distance", distanceFromHub);
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