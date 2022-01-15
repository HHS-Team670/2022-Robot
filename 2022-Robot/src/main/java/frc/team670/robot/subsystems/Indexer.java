package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangNotifications;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.motorcontroller.TalonSRXFactory;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents the ball indexer subsystem, which tracks and stores up to 4 balls.
 * 
 * @author pallavidas, aadityaraj
 */

public class Indexer extends MustangSubsystemBase {

    private Conveyor conveyor; // we don't actually use this; can remove it from construc
    private CANSparkMax frontMotor, backMotor;
    private CANEncoder frontEncoder, backEncoder;
    private TalonSRX updraw; // motor

    private boolean[] chamberStates;
    public int[] sensorVals = { 350, 350, 350, 350 };

    private Long updrawStartTime;

    private int chamberToCheck = -1;

    private double UPDRAW_SPEED = -0.9;
    private double INDEXER_SPEED = 0.35;

    private int isUpdrawingCount = 0;
    private int totalNumBalls = 0;
    private SerialPort serialport;

    // Updraw normally at around/below 9A when it's just spinning, peaks when ball
    // touches
    private static final int UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT = 9;
    private static final int UPDRAW_PEAK_CURRENT_LIMIT = 15;

    public Indexer(Conveyor conveyor) {
        super();

        frontMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.FRONT_MOTOR, Motor_Type.NEO_550);
        backMotor = SparkMAXFactory.buildFactorySparkMAX(RobotMap.BACK_MOTOR, Motor_Type.NEO_550);
        frontEncoder = frontMotor.getEncoder();
        backEncoder = backMotor.getEncoder();
        this.conveyor = conveyor;

        frontMotor.setIdleMode(IdleMode.kBrake);
        backMotor.setIdleMode(IdleMode.kBrake);

        // Updraw should be inverted
        this.updraw = TalonSRXFactory.buildFactoryTalonSRX(RobotMap.UPDRAW_SPINNER, false);

        chamberStates = new boolean[4];

        updraw.setNeutralMode(NeutralMode.Coast); // free, nothing holding instead of brake

        updraw.configContinuousCurrentLimit(UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT);
        updraw.configPeakCurrentLimit(UPDRAW_PEAK_CURRENT_LIMIT);
        updraw.enableCurrentLimit(true);

        updraw.configVoltageCompSaturation(12); // "full output" will now scale to 12 Volts
        updraw.enableVoltageCompensation(true);

        serialport = null; // new SerialPort(9600, SerialPort.Port.kUSB1);
    }

    private void pushGameDataToDashboard() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable("/SmartDashboard");
        NetworkTableEntry gameData = table.getEntry("Balls");
        gameData.setNumber(totalNumBalls);
    }

    public void updateChamberStates() {
        totalNumBalls = 0;
        for (int i = 0; i < sensorVals.length; i++) {
            chamberStates[i] = sensorVals[i] < RobotConstants.MIN_BALL_DETECTED_WIDTH_INDEXER;
            if (chamberStates[i])
                totalNumBalls++;
        }
    }

    public void run(boolean isOuttake) {
        frontMotor.set(INDEXER_SPEED);
        backMotor.set(-1 * INDEXER_SPEED);
        if (isOuttake) {
            updraw.set(ControlMode.PercentOutput, UPDRAW_SPEED);
        }
    }

    public void runReversed(boolean isOuttake) {
        frontMotor.set(-1 * INDEXER_SPEED);
        backMotor.set(INDEXER_SPEED);
        if (isOuttake) {
            updraw.set(ControlMode.PercentOutput, -1 * UPDRAW_SPEED);
        }
    }

    public void stop() {
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }

    public void toggleUpdraw() {
        double c = updraw.getMotorOutputPercent();
        if (MathUtils.doublesEqual(c, 0.0, 0.1)) {
            updraw(false);
        } else {
            stopUpdraw();
        }
    }

    /**
     * Run the uptake, emptying the top chamber of the indexer
     * 
     * @param reversed true to run updraw backwards, false to run normally
     */
    public void updraw(boolean reversed) {
        if (updrawStartTime == null) {
            updrawStartTime = System.currentTimeMillis();
        }
        updraw.set(ControlMode.PercentOutput, reversed ? -0.8 * UPDRAW_SPEED : UPDRAW_SPEED);
    }

    public void stopUpdraw() {
        updraw.set(ControlMode.PercentOutput, 0);
        updrawStartTime = null;
    }

    /**
     * 
     * @return true if the updraw is spinning at close to its target speed for
     *         updraw-ing
     */
    public boolean updrawIsUpToSpeed() {
        double c = updraw.getMotorOutputPercent(); // We can't tell if it's actually up to speed,
                                                   // but we're going off of "is it running"
        if (updrawStartTime == null) {
            return MathUtils.doublesEqual(c, UPDRAW_SPEED, 0.05);
        }

        return System.currentTimeMillis() >= updrawStartTime + 500;
    }

    @Override
    public HealthState checkHealth() {
        // if either the rotator or updraw breaks, we can't use the indexer anymore.
        // Same deal if the indexer is jammed (but that's recoverable).
        CANError frontError = frontMotor.getLastError();
        CANError backError = backMotor.getLastError();

        boolean isFrontError = isSparkMaxErrored((SparkMAXLite) frontMotor);
        boolean isBackError = isSparkMaxErrored((SparkMAXLite) backMotor);
        // boolean isRotatorError = isSparkMaxErrored(rotator);
        boolean isUpdrawError = isPhoenixControllerErrored(updraw);
        if (isUpdrawError || isFrontError || isBackError) {
            MustangNotifications.reportError("RED Errors: front: %s, back: %s", frontError, backError);
            return HealthState.RED;
        }
        // if the ToF sensor, (BallSensor) breaks but nothing else,
        // the next option would be manual control -- not a fatal issue
        updateSensorVals();
        for (int i = 0; i < sensorVals.length; i++) {
            if (sensorVals[i] >= 300) {
                return HealthState.YELLOW;
            }
        }
        return HealthState.GREEN;
    }

    public void moveByPercentOutput(double output) {
        frontMotor.set(output);
        backMotor.set(output);
    }

    public boolean isUpdrawing() {
        double updrawCurrent = updraw.getSupplyCurrent();
        if (updrawCurrent > 0.2) {
            if (updrawCurrent >= UPDRAW_NORMAL_CONTINUOUS_CURRENT_LIMIT) {
                isUpdrawingCount++;
            } else {
                isUpdrawingCount = 0;
            }
            if (isUpdrawingCount >= 3) {
                return true;
            }
        }
        return false;
    }

    public void index() {
        if (!isChamberFull(2) && isBallInConveyor() && chamberToCheck == -1) {
            run(false);
            conveyor.run(false);
            if (isChamberFull(1)) {
                chamberToCheck = 2;
            } else if (isChamberFull(0)) {
                chamberToCheck = 1;
            } else {
                chamberToCheck = 0;
            }

        }
        if (isChamberFull(2) && isBallInConveyor()) {

            conveyor.stop();
        }
        if (chamberToCheck != -1) {
            if (isChamberFull(chamberToCheck)) {
                stop();
                chamberToCheck = -1;
            }
        }
    }

    public void reset() {
        chamberToCheck = -1;
    }

    @Override
    public void mustangPeriodic() {
        updateSensorVals();
        updateChamberStates();
        index();
        pushGameDataToDashboard();
        // logSensorVals();
    }

    private void logSensorVals() {
        Logger.consoleLog("0: %s, 1: %s, 2: %s, 3: %s", sensorVals[0], sensorVals[1], sensorVals[2], sensorVals[3]);
        Logger.consoleLog("Chamber to check: %s", chamberToCheck);
        Logger.consoleLog("Total balls: %s", getTotalNumBalls());
    }

    private void updateSensorVals() {
        if (serialport == null) {
            try {
                serialport = new SerialPort(9600, SerialPort.Port.kUSB1);
            } catch (Exception e) {
                Logger.consoleError(e.getMessage());
            }
        }
        if (serialport != null) {
            try {
                String reading = serialport.readString();
                if (reading.length() > 0) {
                    serialParser(reading);
                    for (int i = 0; i < sensorVals.length; i++) {
                        if (sensorVals[i] >= 300) {
                            serialport.reset();
                        }
                    }
                    // logSensorVals();
                }
            } catch (Exception e) {
                serialport = null;
            }
        }
    }

    private void serialParser(String original) { // sd#.value (from 0 to 255) (# is 0-3)
        try {
            String[] sensorData = original.split("\n");
            for (String x : sensorData) {
                if (x.length() > 0) {
                    String[] sensinfo = x.trim().split("\\.");
                    if (sensinfo.length == 2) {
                        sensorVals[Integer.parseInt(sensinfo[0])] = Integer.parseInt(sensinfo[1]);
                    }
                }
            }
        } catch (Exception e) {
            Logger.consoleError(e.getMessage());
        }
    }

    public boolean isBallInConveyor() {
        return sensorVals[3] < RobotConstants.MIN_BALL_DETECTED_WIDTH_TUNNEL;
    }

    public boolean isChamberFull(int chamber) {
        return chamberStates[chamber];
    }

    public boolean ballInChamber(int chamber) {
        return sensorVals[chamber] < RobotConstants.MIN_BALL_DETECTED_WIDTH_INDEXER;
    }

    public boolean isFrontRunning() {
        return frontMotor.get() != 0;
    }

    public boolean isBackRunning() {
        return backMotor.get() != 0;
    }

    public int getTotalNumBalls() {
        return totalNumBalls;
    }

    public double getFrontMotorSpeed() {
        return frontEncoder.getVelocity();
    }

    public double getBackMotorSpeed() {
        return backEncoder.getVelocity();
    }

    public CANSparkMax getFrontMotor() {
        return frontMotor;
    }

    public CANSparkMax getBackMotor() {
        return backMotor;
    }

    public boolean[] getChamberStates() {
        return chamberStates;
    }
}