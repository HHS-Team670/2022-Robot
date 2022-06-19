package frc.team670.robot.commands.auton;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public class AutonPathWithDelay extends CommandBase implements MustangCommand {

    private double targetTimeMillis;
    private double delaySeconds;
    private MustangCommand path;

    public AutonPathWithDelay(double delay, MustangCommand path) {
        this.delaySeconds = delay;
        this.path = path;
        targetTimeMillis = System.currentTimeMillis() + delay * 1000;
    }

    public void initialize() {
        SmartDashboard.putNumber("target delay seconds", delaySeconds);
        SmartDashboard.putNumber("target delay millis", targetTimeMillis);

    }

    public void execute() {
        //SmartDashboard.putNumber("current time millis", System.currentTimeMillis());
        if ((int) ((targetTimeMillis - System.currentTimeMillis()) / 1000.0) >= 0) {
            SmartDashboard.putNumber("countdown", (int) ((targetTimeMillis - System.currentTimeMillis()) / 1000.0));
        }
    }

    public boolean isFinished() {
        // Logger.consoleLog("VALUE OF NUMBER: " + (int)( (targetTimeMillis -
        // System.currentTimeMillis()) / 1000.0));
        if ((int) ((targetTimeMillis - System.currentTimeMillis()) / 1000.0) <= 0) {
            return true;
        }
        return false;
    }

    public void end(boolean interrupted) {
        MustangScheduler.getInstance().schedule(path);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return path.getHealthRequirements();
    }

}
