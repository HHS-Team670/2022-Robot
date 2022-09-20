package frc.team670.robot.commands.routines.intake;

import java.util.HashMap;
import java.util.Map;

// import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Intake;
// import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.ConveyorSystem.Status;

/**
 * Starts ramping up the shooter and runs it
 * 
 * @author arghunter
 */
public class ToggleIntakeAndConveyor extends CommandBase implements MustangCommand {

    private Intake intake;
    private ConveyorSystem conveyor;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private double rpm;

    /**
     * @param intake the intake object
     * @param conveyor the conveyor
     * 
     */
    public ToggleIntakeAndConveyor(Intake intake,ConveyorSystem conveyor) {
        this.intake = intake;
        this.conveyor=conveyor;
        rpm = 0;
        addRequirements(intake,conveyor);
    
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.GREEN);
        healthReqs.put(conveyor, HealthState.GREEN);

    }



    @Override
    public void initialize() {
        if(conveyor.isRunning()&&intake.isRolling()){
            conveyor.stopAll();
            intake.stop();
        }else{
            conveyor.setConveyorMode(Status.INTAKING);
            intake.roll(false);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
