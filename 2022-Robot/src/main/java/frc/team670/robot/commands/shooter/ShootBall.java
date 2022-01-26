package frc.team670.robot.commands.shooter;

import frc.team670.robot.subsystems.*;

import java.util.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;

public class ShootBall extends CommandBase implements MustangCommand {
    private Shooter shooter;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    
    public ShootBall(Shooter s)
    {
        shooter = s;
        addRequirements(s);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        shooter.setRampRate(true);
        shooter.run();
        Logger.consoleLog("StartShooter Initialized");
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
