package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.*;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.subsystems.Intake;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

/**
 * Deploys the intake
 * @author Santan, Armaan
 */
public class DeployIntake extends CommandBase implements MustangCommand {

    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Intake intake;

    
    // Deploys the intake
    public DeployIntake(Intake intake) {
        this.intake = intake;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.GREEN);
        addRequirements(intake);
    }
    
    public void initialize() {
        intake.deployer.deployIntake();
    }

    public void end() {
        intake.deployer.stop();
    }

    public boolean isFinished() {
        return intake.deployer.isAtTarget();
    }

    /*
     * returns the health state of the intake
     */

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}