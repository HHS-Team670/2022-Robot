package frc.team670.robot.commands;

import java.util.Date;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.auton.PutMessage;

public class PutMessageAfterDelay extends SequentialCommandGroup implements MustangCommand{


    private double targetTimeMillis;

    public PutMessageAfterDelay(double delay, String message) {
        targetTimeMillis = System.currentTimeMillis() + delay*1000;


        addCommands(new PutMessage("Delay countdown for " + delay + " seconds"), 
                        new WaitCommand(delay), 
                            new PutMessage(message));
    }

    public void execute() {
        SmartDashboard.putNumber( "countdown", (int)( (targetTimeMillis - System.currentTimeMillis()) / 1000.0));
        
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
