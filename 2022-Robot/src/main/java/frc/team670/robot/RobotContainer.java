/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.*;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.robot.commands.shooter.StartShooter;


public class RobotContainer extends RobotContainerBase {

    private static OI oi = new OI();
    //private static Vision vision = new Vision();
    private static Shooter shooter = new Shooter();

    int i = 0;

    private MustangCommand m_autonomousCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        super();
        addSubsystem(shooter);
      
    }

    public void robotInit() {
      
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public MustangCommand getAutonomousCommand() {
        return null;
    }

    public void autonomousInit() {
      
    }

    public void teleopInit() {
      MustangScheduler.getInstance().schedule(new StartShooter(shooter, false));
      
    }

    @Override
    public void disabled() {
      
    }

    public static Joystick getOperatorController() {
        return OI.getOperatorController();
    }

    public static void rumbleDriverController() {
        notifyDriverController(1.0, 0.3);
    }

    public static void rumbleDriverController(double power, double time) {
        oi.rumbleDriverController(power, time);
    }

    public static void notifyDriverController(double power, double time) {
        oi.rumbleDriverController(power, time);
    }

    public static MustangController getDriverController() {
        return OI.getDriverController();
    }

    public void periodic() {
        //break1.sendBeamBreakDataToDashboard();
    }

}