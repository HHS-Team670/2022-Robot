package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.utils.LEDColor;
import frc.team670.robot.RobotContainer;

/**
 * Represents the LED light strips on the robot.
 * These LEDs change color depending on the selected autonomous path, the number of cargo in the conveyor, and the shooting status.
 * 
 * @author AkshatAdsule, LakshBhambhani
 */
public class LEDs extends LEDSubsystem {

    private Shooter shooter;
    private Intake intake;
    private ConveyorSystem conveyors;
    private ClimberSystem climbers;

    private LEDColor allianceColor, oppositeAllianceColor;

    public LEDs(int port, int startIndex, int endIndex, Shooter shooter, Intake intake, ConveyorSystem conveyors,
            ClimberSystem climbers) {
        super(port, startIndex, endIndex);
        this.shooter = shooter;
        this.intake = intake;
        this.conveyors = conveyors;
        this.climbers = climbers;
    }

    public void setAllianceColors(LEDColor alliance, LEDColor oppositeAlliance) {
        //CL0E -> 0boT1c5_
        // Go to the class that deploys the intake
        this.allianceColor = alliance;
        this.oppositeAllianceColor = oppositeAlliance;
    }

    @Override
    public void mustangPeriodic() {
        if (DriverStation.isDisabled()) {
            String path = RobotContainer.getAutoChooser().getSelected().toString();
            if(path.contains("FourBallPath"))
                solid(LEDColor.GREEN.dimmer());
            else if(path.contains("TwoBallPath"))
                solid(LEDColor.BLUE.dimmer());
            else
                rainbow(false);
        } else if (!isBlinking) {
            if (climbers.isRobotClimbing())
                blink(LEDColor.PURPLE.dimmer());
            else if(shooter.foundTarget())
                blink(LEDColor.GREEN.dimmer());
            else if (conveyors.getBallCount() == 2) // show amount of balls in conveyor
                solid(oppositeAllianceColor.dimmer());
            else if (shooter.getVelocity() > 0) // shooter is shooting
                if (shooter.isShooting())
                    blink(oppositeAllianceColor.dimmer(), 10);
            else if (conveyors.getBallCount() == 1) // show amount of balls in conveyor
                progressBar(allianceColor.dimmer(), oppositeAllianceColor.dimmer(), 0.5);
            else if (intake.isRolling()) // intake is running
                blink(allianceColor.dimmer(), 10);
            else
                rainbow(false);
        }

        super.mustangPeriodic(); // to handle blinks and setting the led state
    }

}
