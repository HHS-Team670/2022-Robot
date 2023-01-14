package frc.team670.robot.commands.routines.drivebase;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;

public class AutoLevel extends CommandBase implements MustangCommand {
    private DriveBase driveBase;
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Timer level_timer;
    private Timer error_timer;
    private double LEVEL_CHECK_PERIOD = 5;
    private double ERROR_QUIT_PERIOD = 3;
    private double target = 0;
    private double ALLOWED_ERROR = 0.5;
    private double ERROR_PITCH = 20;

    public AutoLevel(DriveBase driveBase) {
        this.driveBase = driveBase;

        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(driveBase, HealthState.GREEN);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    @Override
    public void execute() {
        double pitch = driveBase.getPitch();
        SmartDashboard.putNumber("Pitch: ", pitch);
        SmartDashboard.putNumber("Leveled time: ", level_timer.get());
        SmartDashboard.putNumber("Error time: ", error_timer.get());

        double error = (target - pitch);

        if (Math.abs(error) < ALLOWED_ERROR) { // check if level.
            level_timer.start(); // noop if timer already running, which is what we want
            driveBase.stop(); // stop motors?
            return;
        }
        level_timer.stop();
        level_timer.reset();

        if (Math.abs(error) > ERROR_PITCH) { //check if error (ramp is held down by another robot?)
            error_timer.start();
            return;
        }
        error_timer.stop();
        error_timer.reset();

        //begin correction
        double kp = 0.05; //use pid code?
        double adjustment = MathUtil.clamp(error * kp, -1, 1);
        driveBase.curvatureDrive(adjustment, 0, false);
    }

    @Override
    public boolean isFinished() {
        return level_timer.hasElapsed(LEVEL_CHECK_PERIOD) || error_timer.hasElapsed(ERROR_QUIT_PERIOD); // level or error
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.stop(); // idk if we want this here but for now imma go with yes
    }
}
