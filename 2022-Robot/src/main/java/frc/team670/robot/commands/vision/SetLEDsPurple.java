package frc.team670.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.LEDs;

public class SetLEDsPurple extends InstantCommand{
    LEDs led;


    public SetLEDsPurple(LEDs led) {
        //led.setColorPurple();
        this.led = led;
    }
    
    @Override
    public void initialize(){
        led.setColorPurple();
    }

    
}