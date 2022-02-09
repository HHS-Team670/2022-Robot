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
 * Intakes n balls where N is specified by whoever calls it
 * @author Sanatan
 */

 
public class IntakeNBalls extends SequentialCommandGroup implements MustangCommand {

    private Intake intake;
    private ConveyorSystem conveyor;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    // Sets up everything

    public IntakeNBalls(Intake intake, ConveyorSystem conveyor, int nBalls) {
        this.intake = intake;
        this.conveyor = conveyor;
        addRequirements(this.intake);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(this.intake, HealthState.GREEN);
        addCommands(
                new DeployIntake(this.intake),
                new RunIntakeWithConveyor(this.intake, this.conveyor),
                new StopAtNBalls(this.conveyor, nBalls),
                new ParallelCommandGroup(new StopIntake(this.intake), new StopConveyor(conveyor)));
    }

    // Returns health state

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
