package frc.team670.robot.commands.routines.intake;

import java.util.HashMap;
import java.util.Map;

// import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.commands.routines.StopAll;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Shooter;
// import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.ConveyorSystem.Status;

/**
 * Starts ramping up the shooter and runs it
 * 
 * @author arghunter
 */
public class ToggleIntakeAndConveyor extends InstantCommand implements MustangCommand {

    private Intake intake;
    private ConveyorSystem conveyor;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Shooter shooter;

    /**
     * @param intake   the intake object
     * @param conveyor the conveyor
     * 
     */
    public ToggleIntakeAndConveyor(Intake intake, ConveyorSystem conveyor, Shooter shooter) {
        super();
        
        this.intake = intake;
        this.conveyor = conveyor;
        this.shooter = shooter;

        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);

    }

    @Override
    public void initialize() {
        /**if (conveyor.isRunning() || intake.isRolling()) {
            conveyor.stopAll();
            intake.stop();
        } else {
            conveyor.setConveyorMode(Status.INTAKING);
            intake.roll(false);
        }*/
        Logger.consoleLog("Ran ToggleIntake");
        if(conveyor.isRunning() || intake.isRolling()) {
            MustangScheduler.getInstance().schedule(new StopAll(intake, conveyor, shooter));
        } else {
            MustangScheduler.getInstance().schedule(new RunIntakeWithConveyor(intake, conveyor));
        }
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
