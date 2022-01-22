package frc.team670.robot.commands.climb;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class ClimberBaseCommand extends CommandBase implements MustangCommand {
    protected Climber climber;
    protected static final double MAX_EXTENDING_HEIGHT_CM = 66.24; // TODO: change this later
    protected Map<MustangSubsystemBase, HealthState> healthReqs;

    public ClimberBaseCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(climber, HealthState.GREEN);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climber.stop();
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}
