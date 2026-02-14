
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX left;
    private final TalonFX center;
    private final TalonFX right;
    private final TalonFX kicker;
    
    // Request object to avoid allocation in loops
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private double targetRpm = 0.0;
    private double targetRpmCenter = 3500;
    private double targetRpmLeft = 3000;
    private double targetRpmRight = 3000;
    
    // Constants
    private static final double MAX_RPM = 6000.0; 
    private static final double MIN_RPM = 5;
    private static final double RPM_TO_RPS = 1.0 / 60.0;
    private static final double CURRENT_LIMIT = 40.0; // Amps

    public enum Side {
        LEFT,
        CENTER,
        RIGHT;
    }

    private Side selectedSide = Side.LEFT;

    private boolean isShooting = false;
    private boolean isKicking = false;

    public ShooterSubsystem() {

        left = new TalonFX(31);
        center = new TalonFX(32);
        right = new TalonFX(33);

        kicker = new TalonFX(36);

        TalonFXConfiguration controlCfg = new TalonFXConfiguration();
        controlCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        controlCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // Current Limits
        controlCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        controlCfg.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

        controlCfg.Slot0.kP = 0.09;
        controlCfg.Slot0.kI = 0;
        controlCfg.Slot0.kD = 0.001;
        controlCfg.Slot0.kV = 0.12; // ~12V

        TalonFXConfiguration controlCfgRight = new TalonFXConfiguration();
        controlCfgRight.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        controlCfgRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // Current Limits
        controlCfgRight.CurrentLimits.SupplyCurrentLimitEnable = true;
        controlCfgRight.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

        controlCfgRight.Slot0.kP = 0.09;
        controlCfgRight.Slot0.kI = 0;
        controlCfgRight.Slot0.kD = 0.001;
        controlCfgRight.Slot0.kV = 0.12; // ~12V

        left.getConfigurator().apply(controlCfg);
        center.getConfigurator().apply(controlCfg);
        right.getConfigurator().apply(controlCfgRight);
    }

    public void setSelected(Side side) {
        selectedSide = side;
    }

    private void setTargetRpmCenter(double rpm) {
        if (rpm > MAX_RPM) rpm = MAX_RPM;
        if (rpm < -MAX_RPM) rpm = -MAX_RPM;
        targetRpmCenter = rpm;
    }

    private void setTargetRpmLeft(double rpm) {
        if (rpm > MAX_RPM) rpm = MAX_RPM;
        if (rpm < -MAX_RPM) rpm = -MAX_RPM;
        targetRpmLeft = rpm;
    }

    private void setTargetRpmRight(double rpm) {
        if (rpm > MAX_RPM) rpm = MAX_RPM;
        if (rpm < -MAX_RPM) rpm = -MAX_RPM;
        targetRpmRight = rpm;
    }

    /**
     * sets targetrpm for all three motors
     * @param rpm
     */
    private void setTargetRpm(double rpm) {
        if (rpm > MAX_RPM) rpm = MAX_RPM;
        if (rpm < -MAX_RPM) rpm = -MAX_RPM;
        targetRpmRight = rpm;
        targetRpmLeft = rpm;
        targetRpmCenter = rpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getTargetRpmCenter() {
        return targetRpmCenter;
    }

    public double getTargetRpmLeft() {
        return targetRpmLeft;
    }

    public double getTargetRpmRight() {
        return targetRpmRight;
    }

    public void increaseSelectedTarget(double deltaRpm) {
        if (selectedSide == Side.LEFT) {
            setTargetRpmLeft(targetRpmLeft + deltaRpm);
        } else if (selectedSide == Side.CENTER) {
            setTargetRpmCenter(targetRpmCenter + deltaRpm);
        } else {
            setTargetRpmRight(targetRpmRight + deltaRpm);
        }
    }

    public void decreaseSelectedTarget(double deltaRpm) {
        if (selectedSide == Side.LEFT) {
            setTargetRpmLeft(targetRpmLeft - deltaRpm);
        } else if (selectedSide == Side.CENTER) {
            setTargetRpmCenter(targetRpmCenter - deltaRpm);
        } else {
            setTargetRpmRight(targetRpmRight - deltaRpm);
        }
    }

    public void increaseTargetRpm(double deltaRpm) {
        setTargetRpmCenter(targetRpmCenter + deltaRpm);
        setTargetRpmLeft(targetRpmLeft + deltaRpm);
        setTargetRpmRight(targetRpmRight + deltaRpm);
    }

    public void decreaseTargetRpm(double deltaRpm) {
        setTargetRpmCenter(targetRpmCenter - deltaRpm);
        setTargetRpmLeft(targetRpmLeft - deltaRpm);
        setTargetRpmRight(targetRpmRight - deltaRpm);
    }

    public void rotateKicker() {
        isKicking = true;
        kicker.set(0.3);
    }

    /**
     * Runs the motor at the specified RPM using closed-loop control.
     * @param rpm Target RPM
     */
    public void rotate(double rpmLeft, double rpmRight, double rpmCenter) {
        isShooting = true;
        // Convert RPM to RPS
        left.setControl(velocityRequest.withVelocity(rpmLeft * RPM_TO_RPS));
        center.setControl(velocityRequest.withVelocity(rpmCenter * RPM_TO_RPS));
        right.setControl(velocityRequest.withVelocity(rpmRight * RPM_TO_RPS));
    }

    public void rotateAtCached() {
        rotate(targetRpmLeft, targetRpmRight, targetRpmCenter);
    }

    public void stopKicker() {
        isKicking = false;
        kicker.stopMotor();
    }

    public void stop() {
        isShooting = false;
        left.stopMotor();
        right.stopMotor();
        center.stopMotor();
    }

    /**
     * @return Current velocity in RPM
     */
    public double getSpeedRpmCenter() {
        return center.getRotorVelocity().getValueAsDouble() * 60.0;
    }

     /**
     * @return Current velocity in RPM
     */
    public double getSpeedRpmLeft() {
        return left.getRotorVelocity().getValueAsDouble() * 60.0;
    }

     /**
     * @return Current velocity in RPM
     */
    public double getSpeedRpmRight() {
        return right.getRotorVelocity().getValueAsDouble() * 60.0;
    }

    public double getShooterSupplyCurrent() {
        return right.getSupplyCurrent().getValueAsDouble() +
               left.getSupplyCurrent().getValueAsDouble() +
               center.getSupplyCurrent().getValueAsDouble();
    }

    public boolean isRunning(double rpm) {
        if (rpm != 0) {
            return true;
        }
        return false;
    }

    public boolean isShooting() {
        return isShooting;
    }

    public boolean isKicking() {
        return isKicking;
    }

    @Override
    public void periodic() {
    }
}