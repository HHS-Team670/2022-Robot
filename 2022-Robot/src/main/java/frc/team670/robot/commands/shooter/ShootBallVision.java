package frc.team670.robot.commands.shooter;

import frc.team670.robot.subsystems.*;

import java.util.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;

public class ShootBallVision extends CommandBase implements MustangCommand {
    private Shooter shooter;
    private Vision vision;
    private double targetRPM;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    
    public ShootBallVision(Vision v, Shooter s)
    {
        shooter = s;
        vision = v;
        addRequirements(s);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        if (vision.getHealth(true) == HealthState.GREEN) {
            double distanceToTarget = vision.getDistanceToTargetM();
            Logger.consoleLog("Shooter distance to target %s", distanceToTarget);
            targetRPM = shooter.getTargetRPMForDistance(distanceToTarget);
        }
        else {
            targetRPM = shooter.getDefaultRPM();
        }
        Logger.consoleLog("Shooter Stage 2 RPM should be %s", targetRPM);
        shooter.setVelocityTarget(targetRPM);
        shooter.setRampRate(true);
    }

    @Override
    public void execute() {
        shooter.run();
    }

    @Override
    public boolean isFinished() {
        return shooter.isUpToSpeed();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}
