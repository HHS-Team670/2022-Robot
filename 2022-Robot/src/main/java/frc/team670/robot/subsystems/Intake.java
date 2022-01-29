package frc.team670.robot.subsystems;

import frc.team670.robot.constants.RobotMap;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;

/*
 * @author Khicken
*/
public class Intake extends MustangSubsystemBase {

  private SparkMAXLite roller;
  private SparkMAXLite deployer;
  
  private boolean isDeployed = false; //TODO: true for testing, change this


  private static final double INTAKE_DEPLOYER_SPEED=1.0;
  private static final double INTAKE_ROLLER_SPEED = 1.0; 
  
  private double INTAKE_PEAK_CURRENT = 35; // Testing
  private int exceededCurrentLimitCount = 0;

  public Intake() {
    // Intake roller should be inverted
    roller = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_ROLLER, Motor_Type.NEO_550);
    deployer = SparkMAXFactory.buildFactorySparkMAX(RobotMap.INTAKE_DEPLOYER, Motor_Type.NEO_550);
    roller.setInverted(true);
    roller.setOpenLoopRampRate(1.0);
  
    
  }

  public boolean isRolling() { 
    return roller.get() != 0;
  }

  public void deploy(boolean isDeployed) {
    this.isDeployed = isDeployed;
    deployer.set(INTAKE_DEPLOYER_SPEED);
    
  }

  public boolean isDeployed() {
    return isDeployed;
  }

  public void roll(boolean reversed) {
     if (isDeployed) {
      if (reversed) {
        roller.set(INTAKE_ROLLER_SPEED * -1);
      } else {
        roller.set(INTAKE_ROLLER_SPEED);
      }
     }
  }

  public boolean isJammed(){
    double intakeCurrent = roller.getOutputCurrent();
    if (intakeCurrent > 0.2){
      if (intakeCurrent >= INTAKE_PEAK_CURRENT) {
        exceededCurrentLimitCount++;
      } else {
        exceededCurrentLimitCount = 0;
      }
      if (exceededCurrentLimitCount >= 4){ // 4 consecutive readings higher than peak
        return true;
      }
    }

    return false;

  }

  public void stop() {
    roller.stopMotor();
    retractIntake();  
  }

  public void retractIntake () {
    deployer.set(INTAKE_DEPLOYER_SPEED * -1);
  }

  /**
   * @return RED if the roller has issues, or the intake isn't deployed but the
   *     pneumatics have issues
   */
  @Override
  public HealthState checkHealth() {
    if (roller == null || isSparkMaxErrored(roller)) {
      return HealthState.RED;
    }
    if (deployer == null) {
      if (isDeployed) {

        return HealthState.YELLOW;
      }

      return HealthState.RED;
    }
    return HealthState.GREEN;
  }

  @Override
  public void mustangPeriodic() {
    checkHealth();
  }

}