package frc.team670.robot.commands.intake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.Intake;

/**
 * Runs the intake, controls the direction based on whether or not it is jammed
 */
public class RunIntake extends CommandBase implements MustangCommand {

    Map<MustangSubsystemBase, HealthState> healthReqs;
    private boolean reversed;
    private Intake intake;
    private int countWasJammed;

    /**
     * @param reversed true to run the intake in reverse (out), 
     * false to run it normally (in)
     */
    public RunIntake(boolean reversed, Intake intake) {
        this.reversed = reversed;
        this.intake = intake;
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(intake, HealthState.YELLOW);
        addRequirements(intake);
        countWasJammed = 0;
    }
    /*
    starts to roll the intake
    */
    public void initialize() {
        intake.roll(reversed);
    }

    /*
    If it's jammed it rolls the other way to unjam it, otherwise it rolls normally
    */
    public void execute() {
        SmartDashboard.putNumber("count jammed", countWasJammed);
        if (intake.isJammed()) {
            countWasJammed = Intake.JAMMED_COUNT_DEF;
        }
        if (countWasJammed > 0) {
            intake.roll(!reversed);
            countWasJammed--;
        } else {
            intake.roll(reversed);
        }
    }

    /* 
    returns the health state
    */
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}