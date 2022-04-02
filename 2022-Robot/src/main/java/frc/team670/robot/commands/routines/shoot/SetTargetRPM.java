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
import frc.team670.robot.constants.HubType;
import frc.team670.robot.subsystems.Shooter;

//only shoots when the robot is within a desired location
public class SetTargetRPM extends CommandBase implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Shooter shooter;
    private double rpm = shooter.getDefaultRPM();

    //see Shooter.setRPMForDistance(double distance, String hubType) for valid hubTypes
    public SetTargetRPM(Shooter shooter, double rpm) {
     
      this.shooter = shooter;
      this.rpm = rpm;
     
      healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
      healthReqs.put(shooter, HealthState.GREEN);
    } 

    @Override
    public void initialize() {
      // shooter.setRPMForDistance(distanceFromHub); 
      // SmartDashboard.putNumber("target distance", distanceFromHub);
      shooter.setTargetRPM(rpm);
      shooter.run();
    }

    // /**
    // * Determines whether or not the robot is within the acceptable margin from the target pose.
    // * @return true if the current pose is within the error, false otherwise
    // */
    // @Override
    // public boolean isFinished(){
    // }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
      return healthReqs;
    }
}