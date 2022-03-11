package frc.team670.robot.commands.auton;

import java.util.Map;

import javax.management.InstanceAlreadyExistsException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public class PutMessage extends SequentialCommandGroup implements MustangCommand{
    
    String message;

    public PutMessage(String message) {
        this.message = message;
    }

    public void initialize() {
        SmartDashboard.putString("message", message);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}