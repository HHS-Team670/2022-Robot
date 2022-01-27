package frc.team670.robot.commands.shooter;

import java.util.HashMap;
import java.util.Map;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public class Shoot extends InstantCommand implements MustangCommand {

    private Shooter shooter;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public Shoot(Shooter shooter){
        this.shooter = shooter;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(shooter, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        shooter.setRampRate(false);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}