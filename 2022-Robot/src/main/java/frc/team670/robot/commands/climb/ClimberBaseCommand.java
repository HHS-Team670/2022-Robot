package frc.team670.robot.commands.climb;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.*;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class ClimberBaseCommand extends CommandBase implements MustangCommand {
    protected Climber climber;
    protected Map<MustangSubsystemBase, HealthState> healthReqs;
    protected TelescopingClimber telescopingClimber;

    public ClimberBaseCommand(Climber climber, boolean straight) {
        this.climber = climber;
        this.telescopingClimber = straight ? climber.straight : climber.oblique;
        addRequirements(climber);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(climber, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        climber.stop();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}
