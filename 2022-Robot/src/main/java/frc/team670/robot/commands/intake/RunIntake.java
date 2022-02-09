package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.ConveyorSystem;
import frc.team670.robot.subsystems.Intake;

/**
 * Runs the intake for 1 ball, unjams it too
 */
public class RunIntake extends CommandBase implements MustangCommand {

    Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean reversed;
    private Intake intake;
    private ConveyorSystem conveyor;

    /**
     * @param reversed true to run the intake in reverse (out), 
     * false to run it normally (in)
     */
    public RunIntake(boolean reversed, Intake intake, ConveyorSystem conveyor) {
        this.reversed = reversed;
        this.intake = intake;
        this.conveyor = conveyor;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.YELLOW);
    }
    
    /*
    starts to roll the intake
    */
    public void initialize() {
        intake.countWasJammed = 0;
        intake.roll(reversed);
    }

    /*
    If it's jammed it rolls the other way to unjam it, otherwise it rolls normally
    */
    public void execute() {
        SmartDashboard.putNumber("count jammed", intake.countWasJammed);
        intake.unjam(reversed);
    }

    public boolean isFinished() {
        if (conveyor.ballCount() >= 1) {
            return true;
        } else {
            return false;
        }
    }

    /* 
    returns the health state
    */
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}