package frc.team670.robot.commands.routines.drivebase;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.mustanglib.utils.Logger;

public class AutoLevel extends CommandBase implements MustangCommand{
DriveBase driveBase;
AHRS gyro = new AHRS(SPI.Port.kMXP);
double target = 0;

    public AutoLevel(DriveBase driveBase){
        this.driveBase = driveBase;
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
        double angle = gyro.getRawGyroX();
        Logger.consoleLog("Pitch: "+angle);
        Logger.consoleLog("to string: "+gyro.toString());

        double kp = 0.0005;

        double adjustment = (target-angle)*kp; //clamp for safety later
        driveBase.curvatureDrive(adjustment, 0, false);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
