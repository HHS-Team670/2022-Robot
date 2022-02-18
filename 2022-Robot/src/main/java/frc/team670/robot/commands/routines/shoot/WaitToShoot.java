package frc.team670.robot.commands.routines.shoot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.commands.shooter.SetRPMTarget;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.commands.shooter.SetRPMTarget;
import frc.team670.robot.commands.shooter.StartShooter;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.subsystems.Shooter;

//only shoots when the robot is within a desired location
public class WaitToShoot extends CommandBase implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private Shooter shooter;
    private ConveyorSystem conveyor;
    private Pose2d target;
    private double error;
    private double distanceFromHub;
  
    //error is in meters
    public WaitToShoot(DriveBase driveBase, ConveyorSystem conveyorSystem, Shooter shooter, Pose2d targetPose, double errorInMeters) {
      double distanceX = target.getX() - FieldConstants.HUB_X_POSITION_METERS;
      double distanceY = target.getY() - FieldConstants.HUB_Y_POSITION_METERS;
      distanceFromHub = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2));
      healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
      healthReqs.put(conveyorSystem, HealthState.GREEN);
      healthReqs.put(shooter, HealthState.GREEN);

      this.driveBase = driveBase;
      this.conveyor = conveyor;
      this.target = targetPose;
      this.error = errorInMeters;
      this.shooter = shooter;
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
      
      if(distance < this.error){
        addCommands(

        );
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