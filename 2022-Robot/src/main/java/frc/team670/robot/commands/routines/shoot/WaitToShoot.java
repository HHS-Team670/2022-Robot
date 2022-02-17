package frc.team670.robot.commands.routines.shoot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;
import frc.team670.robot.commands.routines.intake.RunIntakeWithConveyor;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;

//only shoots when the robot is within a desired location
public class WaitToShoot extends SequentialCommandGroup implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private DriveBase driveBase;
    private Pose2d target;
    private double errorX, errorY;
  
    //error is in meters
    public WaitToShoot(DriveBase driveBase, ConveyorSystem conveyorSystem, Shooter shooter, Intake intake, Pose2d targetPose, double errorX, double errorY) {      
      healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
      healthReqs.put(conveyorSystem, HealthState.GREEN);
      healthReqs.put(shooter, HealthState.GREEN);

      this.driveBase = driveBase;
      this.target = targetPose;
      this.errorX = errorX;
      this.errorY = errorY;
      addCommands(

        
        new StartShooterByDistance(...),
        // new ShootAllBalls(conveyorSystem, shooter)
      );
    } 

    @Override
    public boolean isFinished(){
      if ((driveBase.getPose().getX() > target.getX() - errorX) && (driveBase.getPose().getX() < target.getX() + errorX) 
      && (driveBase.getPose().getY() > target.getY() - errorY) && (driveBase.getPose().getY() < target.getY() + errorY)){
        return true;
      } else {
        return false;
      }
    }



    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
      return healthReqs;
    }
}