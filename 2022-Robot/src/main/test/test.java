import static org.junit.*;
import static org.junit.Assert.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.Constants.IntakeConstants;
import frc.team670.robot.subsystems.Shooter;
import frc.team670.robot.subsystems.Vision;

import org.junit.*;

public class test {
    //declaring Shooter and Vision
    Shooter shooter;
    Vision vision;
    

    @Before//this method runs before every test
    public void setup() {
        //initializing shooter and vision
        vision = new Vision();
        shooter = new Shooter(vision);
    }

    @After//runs after every test
    public void shutdown() throws Exception{
        shooter.stop();//stops the shooter
    }

    @Test // marks this method as a test
    public void doesntShoot() {
       // shooter.setRampRate(false); //hopefully shoots the robot?
        Assert.assertTrue(shooter.isShooting(),false);
    }

    @Test// test if the robot shoots
    public void robotIsShooting() {
        shooter.setRampRate(false); //hopefully shoots the robot?
        Assert.assertTrue(shooter.isShooting(),true);
    }

    @Test
    public void testSpeed() {
        Assert.assertEquals(/*expected value*/, shooter.getTargetRPMForDistance(distance), /* double delta*/);       
    }

}
