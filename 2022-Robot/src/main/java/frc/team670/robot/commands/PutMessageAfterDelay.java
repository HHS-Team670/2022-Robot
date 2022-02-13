package frc.team670.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;

public class PutMessageAfterDelay extends WaitCommand implements MustangCommand{


    private double targetTimeMillis;
    private double delaySeconds;
    private String message;

    public PutMessageAfterDelay(double delay, String message) {
        super(delay);
        this.message = message;
        this.delaySeconds = delay;
        targetTimeMillis = System.currentTimeMillis() + delay*1000;
    }

    public void initialize() {
        Logger.consoleLog("Delay countdown for " + delaySeconds + " seconds");
    }

    public void execute() {
        SmartDashboard.putNumber( "countdown", (int)( (targetTimeMillis - System.currentTimeMillis()) / 1000.0));
        
    }

    public void end() {
        SmartDashboard.putString("message", message);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
