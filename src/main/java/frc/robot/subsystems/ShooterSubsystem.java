
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

    private final LinearActuator actuator1;
    private final LinearActuator actuator2;
    
    // Request object to avoid allocation in loops
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private double targetRpm = 0.0;
    private double targetRpmCenter = 0.0;
    private double targetRpmLeft = 0.0;
    private double targetRpmRight = 0.0;
    
    // Constants
    private static final double MAX_RPM = 6000.0; 
    private static final double MIN_RPM = 5;
    private static final double RPM_TO_RPS = 1.0 / 60.0;
    private static final double CURRENT_LIMIT = 40.0; // Amps

    public ShooterSubsystem() {

        actuator1 = new LinearActuator(0, 100, 2);
        actuator2 = new LinearActuator(2, 100, 2);

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

        controlCfg.Slot0.kP = 0.11;
        controlCfg.Slot0.kI = 0.5;
        controlCfg.Slot0.kD = 0.0001;
        controlCfg.Slot0.kV = 0.12; // ~12V

        TalonFXConfiguration controlCfgRight = new TalonFXConfiguration();
        controlCfgRight.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        controlCfgRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // Current Limits
        controlCfgRight.CurrentLimits.SupplyCurrentLimitEnable = true;
        controlCfgRight.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

        controlCfgRight.Slot0.kP = 0.11;
        controlCfgRight.Slot0.kI = 0.5;
        controlCfgRight.Slot0.kD = 0.0001;
        controlCfgRight.Slot0.kV = 0.12; // ~12V

        left.getConfigurator().apply(controlCfg);
        center.getConfigurator().apply(controlCfg);
        right.getConfigurator().apply(controlCfgRight);
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

    public void increaseTargetRpmRight(double deltaRpm) {
        setTargetRpmRight(targetRpmRight + deltaRpm);
    }

    public void decreaseTargetRpmRight(double deltaRpm) {
        setTargetRpmRight(targetRpmRight - deltaRpm);
    }

    public void increaseTargetRpmLeft(double deltaRpm) {
        setTargetRpmLeft(targetRpmLeft + deltaRpm);
    }

    public void decreaseTargetRpmLeft(double deltaRpm) {
        setTargetRpmLeft(targetRpmLeft - deltaRpm);
    }

    public void increaseTargetRpmCenter(double deltaRpm) {
        setTargetRpmCenter(targetRpmCenter + deltaRpm);
    }

    public void decreaseTargetRpmCenter(double deltaRpm) {
        setTargetRpmCenter(targetRpmCenter - deltaRpm);
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

    /**
     * Runs the motor at the specified RPM using closed-loop control.
     * @param rpm Target RPM
     */
    public void rotate(double rpmLeft, double rpmRight, double rpmCenter) {
        // Convert RPM to RPS
        kicker.set(0.3);
        left.setControl(velocityRequest.withVelocity(rpmLeft * RPM_TO_RPS));
        center.setControl(velocityRequest.withVelocity(rpmCenter * RPM_TO_RPS));
        right.setControl(velocityRequest.withVelocity(rpmRight * RPM_TO_RPS));
    }

    public void rotateAtCached() {
        rotate(targetRpmLeft, targetRpmRight, targetRpmCenter);
    }

    public void stop() {
        kicker.stopMotor();
        left.stopMotor();
        right.stopMotor();
        center.stopMotor();
    }

    public void raise() {
        actuator1.setSpeed(0.6);
        actuator2.setSpeed(0.6);
    }

    public void lower() {
        actuator1.setSpeed(-0.6);
        actuator2.setSpeed(-0.6);
    }

    public void stopActuator() {
        actuator1.setDisabled();
        actuator2.setDisabled();
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

    @Override
    public void periodic() {
    }
}