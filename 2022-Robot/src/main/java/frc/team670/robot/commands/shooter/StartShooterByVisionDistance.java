package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;
/*
Uses vision to get the target distance and predicts the RPM based off that
If vision is broken, it just uses the default RPM
*/
public class StartShooterByVisionDistance extends CommandBase implements MustangCommand {

    private Shooter shooter;
    private Vision vision;

    /*
    * Toggle: If the shooter is running then it stops it.
    * If the shotter is stopping then it runs it.
    */
    private boolean toggle;

    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public StartShooterByVisionDistance(Shooter shooter, Vision vision, boolean toggle){
        this.shooter = shooter;
        this.vision = vision;
        this.toggle = toggle;
        addRequirements(shooter);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        if(toggle) {
            if (MathUtils.doublesEqual(0.0, shooter.getVelocity(), 10)) {
                setRPM();
            } else {
                shooter.stop();
            }
        }else {
            setRPM();
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.isUpToSpeed();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }


    /*
    *Method for setting RPM:
    *If vision works, it gets the distance to target from vision, 
    *predicts the RPM based off the distance, 
    *and sets that as the Target RPM
    *If vision doesn't work, just sets the default RPM as the target RPM
    */
    private void setRPM() {
        if(vision.getHealth(true) == HealthState.GREEN) {
            double distanceToTarget = vision.getDistanceToTargetM();
            shooter.setRPMForDistance(distanceToTarget);
        }else{
            shooter.setTargetRPM(shooter.getDefaultRPM());
        }
        shooter.run();
    }
    
}