package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.robot.RobotContainer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;


/**
 * Represents the Color sensor at the intake of the robot
 * 
 *
 * @author Edward
 */
public class ColorSensor extends MustangSubsystemBase {


    private Intake intake;
    private ConveyorSystem conveyors;

    private Color detectedColor;
    private final ColorSensorV3 m_colorSensor;

    public LEDs(I2C.Port i2cPort, port,Intake intake, ConveyorSystem conveyors) {
       m_colorSensor = new ColorSensorV3(i2cPort);

//        super(port);
        this.intake = intake;
        this.conveyors = conveyors;
    }

    public void setAllianceColors(LEDColor alliance, LEDColor oppositeAlliance) {
        this.allianceColor = alliance;
        this.oppositeAllianceColor = oppositeAlliance;
    }
    @Override
    public void mustanglib()
    {
        detectedColor = m_colorSensor.getColor();
        //double IR = m_colorSensor.getIR();
        string text;
        if(detectedColor.red > detectedColor.blue)
        {
            text = "Red"
        }else
        {
            text = "Blue"
        }

        SmartDashboard.putString("Color: ",text);
        // SmartDashboard.putNumber("Red", detectedColor.red);
        // SmartDashboard.putNumber("Green", detectedColor.green);
        // SmartDashboard.putNumber("Blue", detectedColor.blue);
        //SmartDashboard.putNumber("IR", IR);
        //int proximity = m_colorSensor.getProximity();
        //SmartDashboard.putNumber("Proximity", proximity);
    }

    // @Override
    // public void mustangPeriodic() {
    //     if (DriverStation.isDisabled()) {
    //         String path = RobotContainer.getAutoChooser().getSelected().toString();
    //         if(path.contains("FourBallPath"))
    //             solid(LEDColor.GREEN.dimmer());
    //         else if(path.contains("TwoBallPath"))
    //             solid(LEDColor.BLUE.dimmer());
    //         else
    //             rainbow(false);
    //     } else if (!isBlinking) {
    //         if (climbers.isRobotClimbing())
    //             blink(LEDColor.PURPLE.dimmer());
    //         else if(shooter.foundTarget())
    //             blink(LEDColor.GREEN.dimmer());
    //         else if (conveyors.getBallCount() == 2) // show amount of balls in conveyor
    //             solid(oppositeAllianceColor.dimmer());
    //         else if (shooter.getVelocity() > 0) // shooter is shooting
    //             if (shooter.isShooting())
    //                 blink(oppositeAllianceColor.dimmer(), 10);
    //         else if (conveyors.getBallCount() == 1) // show amount of balls in conveyor
    //             progressBar(allianceColor.dimmer(), oppositeAllianceColor.dimmer(), 0.5);
    //         else if (intake.isRolling()) // intake is running
    //             blink(allianceColor.dimmer(), 10);
    //         else
    //             rainbow(false);
    //     }

    //     super.mustangPeriodic(); // to handle blinks and setting the led state
    // }

}
