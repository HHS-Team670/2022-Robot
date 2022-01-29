package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

public class ToggleShooter extends InstantCommand implements MustangCommand {

    private Shooter shooter;
    private Vision vision;

    private double targetRPM;

    public ToggleShooter(Shooter shooter, Vision vision) {
        this.shooter = shooter;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        super.initialize();
        if (MathUtils.doublesEqual(0.0, shooter.getVelocity(), 10)) {
            if(vision.getHealth(true) == HealthState.GREEN){
                double distanceToTarget = vision.getDistanceToTargetM();
                shooter.setRPMForDistance(distanceToTarget);
            }
            else{
                targetRPM = shooter.getDefaultRPM();
                Logger.consoleLog("Shooter RPM should be %s", targetRPM);
                shooter.setTargetRPM(targetRPM);        
            }
            shooter.run();
        } else {
            shooter.stop();
        }
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }
    
}