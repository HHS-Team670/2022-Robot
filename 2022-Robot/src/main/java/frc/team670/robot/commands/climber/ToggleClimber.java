package frc.team670.robot.commands.climber;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ClimberSystem;
import frc.team670.robot.subsystems.ClimberSystem.Climber;
import frc.team670.robot.subsystems.ClimberSystem.Level;

/**
 * Lower the Climber mechanism.
 */
public class ToggleClimber extends CommandBase implements MustangCommand {

    private ClimberSystem.Climber climber;
    private HashMap<MustangSubsystemBase, HealthState> healthReqs;
    private ClimberSystem.Level level;

    public ToggleClimber(Climber climber, ClimberSystem.Level level) {
        this.climber = climber;
        this.level = level;
        addRequirements(climber);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(climber, HealthState.GREEN);
    }

    @Override
    public void initialize() {
        if (climber.getCurrentLevel() == Level.RETRACTED) {
            climber.climb(level);
        } else {
            climber.retract();
        }

    }

    @Override
    public boolean isFinished() {
        return climber.isAtTarget() || climber.isLimitSwitchTripped();
        // return climber.isLimitSwitchTripped();

    }

    public void end() {
        climber.stop();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}
