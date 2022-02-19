package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

/**
 * Starts ramping up the shooter and runs it
 * 
 * @author wilsonpotato, palldas
 */
public class StartShooter extends CommandBase implements MustangCommand {

    private Shooter shooter;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean useDynamicSpeed;
    private Vision vision;

    /**
     * @param shooter         the shooter object
     * @param useDynamicSpeed if the user wants to use vision
     */
    public StartShooter(Shooter shooter, boolean useDynamicSpeed, Vision vision) {
        this.shooter = shooter;
        this.useDynamicSpeed = useDynamicSpeed;
        addRequirements(shooter);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
        this.vision = vision;
    }

    /**
     * 
     * @param shooter the shooter object
     */
    public StartShooter(Shooter shooter) {
        this(shooter, false, null);
    }

    @Override
    public void initialize() {
        setRPM();
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

    /**
     * If vision works, it gets the distance to target from vision,
     * predicts the RPM based off the distance,
     * and sets that as the Target RPM
     * If vision doesn't work, it tries to use the ultrasonic sensors
     * If that doesn't work either, then it will run the shooter at default speed
     */

    private void setRPM() {
        double targetRPM = shooter.getDefaultRPM();
        if (useDynamicSpeed) {
            double distanceToTarget;
            if (vision.getHealth(true) == HealthState.GREEN) {
                distanceToTarget = vision.getDistanceToTargetM();
                
            } 
            else{
                distanceToTarget = shooter.getUltrasonicDistanceInMeters();
            }
            if (distanceToTarget < shooter.getMinHighDistanceInMeter()) {
                targetRPM = shooter.getTargetRPMForLowGoalDistance(distanceToTarget);
            } else {
                targetRPM = shooter.getTargetRPMForHighGoalDistance(distanceToTarget);
            }
        }
        shooter.setTargetRPM(targetRPM);

    }

}