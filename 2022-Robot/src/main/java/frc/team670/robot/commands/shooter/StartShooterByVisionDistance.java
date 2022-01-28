package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;
/*
Uses vision to get the target distance and predicts the RPM based off that
If vision is broken, it just uses the default RPM
*/
public class StartShooterByVisionDistance extends CommandBase implements MustangCommand {

    private Shooter shooter;
    private Vision vision;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public StartShooterByVisionDistance(Shooter shooter, Vision vision){
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        if(vision.getHealth(true) == HealthState.GREEN){
            double distanceToTarget = vision.getDistanceToTargetM();
            shooter.setRPMForDistance(distanceToTarget);
        }
        else{
            double targetRPM = shooter.getDefaultRPM();
            Logger.consoleLog("Shooter RPM should be %s", targetRPM);
            shooter.setTargetRPM(targetRPM);
        }
        shooter.setRampRate(true);
        shooter.run();
    }

    @Override
    public boolean isFinished() {
        return shooter.isUpToSpeed();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
    
}