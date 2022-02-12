package frc.team670.robot.commands.teleop;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.subsystems.DriveBase;

/**
 *  @author Rathik, Ethan Chang, Aaditya R, Akshat Adzule, Benjamin
 */
public class XboxFieldOrientedDrive extends CommandBase implements MustangCommand {

    private MustangController xbox = new MustangController(0);
    private DriveBase driveBase;
    private NavX navX;
    public final double maxSpeed = 0.5;

    public XboxFieldOrientedDrive(DriveBase driveBase, NavX navX, MustangController xbox) {
        this.driveBase = driveBase;
        this.navX = navX;
        this.xbox = xbox;
        addRequirements(driveBase);
    }

    public void initialize() {
        SmartDashboard.putNumber("Strafe clamp", 0.00);
        SmartDashboard.putNumber("Forward clamp", 0.00);
    }

    @Override
    public void execute() {
        

       
        // SmartDashboard.putNumber("Heading angle", headingAngle);

        // double[] speeds = FieldOrientedDriveOmni.getComponentSpeeds(xSpeed, ySpeed, headingAngle);

        // driveBase.setCenterDrive(speeds[1]);
        // driveBase.curvatureDrive(speeds[0], 0, driveBase.isQuickTurnPressed());

        // get x and y components of joystick push
        double xSpeed = xbox.getLeftStickX();
        double ySpeed = xbox.getLeftStickY();   

        if (xbox.getAButton()) {
            ySpeed = maxSpeed;
        }
        if (xbox.getBButton()) {
            xSpeed = maxSpeed;
        }
        if (xbox.getYButton()) {
            ySpeed = -maxSpeed;
        } 
        if (xbox.getXButton()) {
            xSpeed = -maxSpeed;
        }
        if (xbox.getLeftBumper()) {
            xSpeed = 0;
            ySpeed = 0;
        }
        
        
        SmartDashboard.putNumber("xboxXSpeed", xSpeed);
        SmartDashboard.putNumber("xboxYSpeed", ySpeed);
        
        // get angle formed by field and robot heading
        double navXAngle = navX.getYawDouble(); 

        SmartDashboard.putNumber("Heading angle", navXAngle);

        // twist from triggers
        // double zRotation = -1 * (xbox.getLeftTriggerAxis()
        // - xbox.getRightTriggerAxis());

        

        // twist from right joystick
        double zRotation = xbox.getRightStickX();

        SmartDashboard.putNumber("Twist", zRotation);

        double[] speeds = getComponentSpeeds(xSpeed, ySpeed, navXAngle);

        SmartDashboard.putNumber("Center Speed", speeds[1]);

        SmartDashboard.putNumber("Forward/back speed", speeds[0]);
        //TODO: when centerDrive is going forward/backward, direction is swapped, but normal in strafe.

        // if (speeds[1] > 0) {
        //     zRotation += 0.25;
        // } else if (speeds[1] < 0) {
        //     zRotation -= 0.25;
        // }
        driveBase.setCenterDrive(speeds[1]);
        driveBase.curvatureDrive(speeds[0], zRotation, true);
    }

    private double[] getComponentSpeeds(double xSpeed, double ySpeed, double navXAngle) {
    
    
        // convert to radians
        double angle_radians = navXAngle * Math.PI / 180;

        //frwrd y, strafe x
        double frwd = -xSpeed * Math.sin(angle_radians) + ySpeed * Math.cos(angle_radians);
        double strafe = xSpeed * Math.cos(angle_radians) + ySpeed * Math.sin(angle_radians);

        SmartDashboard.putNumber("Heading radians", angle_radians);

        // clamp strafe speed to avoid rapid switching
        if(Math.abs(strafe) <= SmartDashboard.getNumber("Strafe clamp", 0.00)) {
        strafe = 0;
        }
        if (Math.abs(frwd) <= SmartDashboard.getNumber("Forward clamp", 0.00)) {
            frwd = 0;
        }

        return new double[] {-frwd, strafe};
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
        healthRequirements.put(driveBase, HealthState.YELLOW);
        return healthRequirements;
    }    
}
