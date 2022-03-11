package frc.team670.robot.commands.routines;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.HubType;
import frc.team670.robot.subsystems.Shooter;

//calls WaitCommand (pauses) when robot is within a desired location
public class WaitToPause extends CommandBase implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private Pose2d target;
    private double error;
    // private double distanceFromHub;
    private double duration;

    //error is in meters
    public WaitToPause(DriveBase driveBase, Pose2d targetPose, double errorInMeters, double pauseDuration) {
      this.driveBase = driveBase;
      this.target = targetPose;
      this.error = errorInMeters;
      this.duration = pauseDuration;
     
      // double distanceX = target.getX() - FieldConstants.HUB_X_POSITION_METERS;
      // double distanceY = target.getY() - FieldConstants.HUB_Y_POSITION_METERS;
      // distanceFromHub = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
      healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    } 

    @Override
    public void initialize() {
      // SmartDashboard.putNumber("distanceFromHub", distanceFromHub);
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

    //once the robot is within the right distance, it pauses for [duration] time
    @Override
    public void end(boolean interrupted) {
      andThen(new WaitCommand(duration));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
      return healthReqs;
    }
}