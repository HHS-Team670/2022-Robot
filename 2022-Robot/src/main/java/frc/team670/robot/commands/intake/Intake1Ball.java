package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.*;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.*;
import frc.team670.robot.commands.conveyor.*;


/** 
 * Runs the intake for the time required to intake one ball? Needs to be checked
 * @author Sanatan
 */

 
public class Intake1Ball extends SequentialCommandGroup implements MustangCommand {

    private Intake intake;
    private ConveyorSystem conveyor;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    // Sets up everything

    public Intake1Ball(Intake intake, ConveyorSystem conveyor) {
        this.intake = intake;
        this.conveyor = conveyor;
        addRequirements(this.intake);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(this.intake, HealthState.GREEN);
        addCommands(
                new DeployIntake(this.intake),
                new RunIntakeWithConveyor(this.intake, this.conveyor),
                new ParallelCommandGroup(new StopIntake(this.intake), new StopConveyor(conveyor)));
    }

    // Returns health state

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
