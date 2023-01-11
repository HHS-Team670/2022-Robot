package frc.team670.robot.commands.routines.drivebase;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.constants.RobotMap;

public class AutoLevel extends CommandBase implements MustangCommand {
    DriveBase driveBase;
    NavX m_navx;
    double target = 0;

    public AutoLevel(DriveBase driveBase) {
        this.driveBase = driveBase;
        // m_navx = driveBase.navXMicro;
        m_navx = new NavX(RobotMap.NAVX_PORT);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        // gyro.calibrate();
    }

    @Override
    public void execute() {
        double angle = m_navx.getPitch();
        Logger.consoleLog("Pitch: " + angle);
        // Logger.consoleLog("to string: "+m_navx.toString());

        double kp = 0.05;

        double adjustment = MathUtil.clamp((target - angle) * kp, -1, 1); // check if i called clamp correctly 🤣
        driveBase.curvatureDrive(-adjustment, 0, false); // negative since undo/counteract
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
