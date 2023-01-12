package frc.team670.robot.commands.routines.drivebase;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;

public class AutoLevel extends CommandBase implements MustangCommand {
    private DriveBase driveBase;
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    private double target = 0;

    public AutoLevel(DriveBase driveBase) {
        this.driveBase = driveBase;
        
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(driveBase, HealthState.GREEN);
        //what is addRequirements? do i need it?
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    // @Override
    // public void initialize() {
    //     // probably don't need
    //     // gyro.calibrate();
    // }

    @Override
    public void execute() {
        double angle = driveBase.getPitch();
        SmartDashboard.putString("Pitch: ", "" + angle);

        double kp = 0.05;
        //for fun i want to try doing verlett integration on the gyro and fusion (maybe just a filter) that with the accelrometer to get my own pitch measurement and compare with the getPitch method
        double adjustment = MathUtil.clamp((target - angle) * kp, -1, 1); // check if i called clamp correctly 🤣
        driveBase.curvatureDrive(adjustment, 0, false); // negative adjustment since undo/counteract (update, negative gives oposite direction?)
    }

    @Override
    public boolean isFinished() {
        return super.isFinished(); //return false;
        //maybe do time based (like stop after 5 secs) or angle change based (it has settled down and robot ramp system is basically static equalibrium)
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.stop(); //idk if we want this here but for now imma go with yes
    }
}
