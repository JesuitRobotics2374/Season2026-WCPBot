
package frc.robot.subsystems;

import org.apache.commons.math4.legacy.fitting.PolynomialCurveFitter;
import org.apache.commons.math4.legacy.fitting.WeightedObservedPoints;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.utils.Ballistics;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX left;
    private final TalonFX center;
    private final TalonFX right;
    private final TalonFX kicker;

    private HopperSubsystem m_hopper;
    private DriveSubsystem m_drivetrain;
    private HoodSubsystem m_hood;

    // Request object to avoid allocation in loops
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private double targetRpm = 0.0;
    private double targetRpmCenter = 4100;
    private double targetRpmLeft = 3500;
    private double targetRpmRight = 3500;
    private double targetRpmKicker = 2000;

    // Constants
    private static final double MAX_RPM = 6000.0;
    private static final double MIN_RPM = 5;
    private static final double RPM_TO_RPS = 1.0 / 60.0;
    private static final double CURRENT_LIMIT = 30.0; // Amps
    private static final double KICKER_CURRENT_LIMIT = 60; // Amps

    private boolean doAutoShoot = true; // enabled by default
    private boolean isRed;
    private double MIN_RANGE = 0.5; // meters

    // FIRST VELOCITY, THEN RPM
    // private double[][] leftShooterValues = {{6, 3100}, {6.73, 3450}, {7.72,
    // 4000}};
    // private double[][] centerShooterValues = {{6.26, 3900}, {6.73, 4100}, {7.72,
    // 4900}}; //these are vel based
    // private double[][] rightShooterValues = {{6.26, 3300}, {6.73, 3550}, {7.72,
    // 4250}};

    // distance, then hood pos, then rpms (LCR, kicker)
    private double[][] values = { { 1.35, 0.2, 2800, 3500, 2900, 1800 },
            { 1.65, 0.2, 3000, 3800, 3100, 1800 },
            { 2.00, 0.2, 3200, 4000, 3300, 1800 },
            { 2.48, 0.5, 3500, 4100, 3500, 2000 },
            { 3.00, 0.5, 3400, 4200, 3600, 1800 },
            { 3.50, 0.5, 3700, 4500, 3800, 1800 },
            // { 3.83, 0.5, 3500, 4300, 3500, 1800 } NOT 90% ACCURACY, NOT CONSIDERED
    };

    private double[] leftCoeffs;
    private double[] centerCoeffs;
    private double[] rightCoeffs;

    public enum Side {
        LEFT,
        CENTER,
        RIGHT,
        KICKER;
    }

    private Side selectedSide = Side.LEFT;

    private boolean isShooting = false;
    private boolean isKicking = false;

    public ShooterSubsystem(HopperSubsystem m_hopper, boolean isRed, DriveSubsystem m_drivetrain,
            HoodSubsystem m_hood) {

        this.m_hopper = m_hopper;
        this.m_drivetrain = m_drivetrain;
        this.m_hood = m_hood;

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
        controlCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        controlCfg.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT / 0.75;

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
        controlCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        controlCfg.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT / 0.75;

        controlCfgRight.Slot0.kP = 0.09;
        controlCfgRight.Slot0.kI = 0;
        controlCfgRight.Slot0.kD = 0.001;
        controlCfgRight.Slot0.kV = 0.12; // ~12V

        TalonFXConfiguration controlCfgKicker = new TalonFXConfiguration();
        controlCfgKicker.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        controlCfgKicker.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Current Limits
        controlCfgKicker.CurrentLimits.SupplyCurrentLimitEnable = true;
        controlCfgKicker.CurrentLimits.SupplyCurrentLimit = KICKER_CURRENT_LIMIT;
        controlCfgKicker.CurrentLimits.StatorCurrentLimitEnable = true;
        controlCfgKicker.CurrentLimits.StatorCurrentLimit = KICKER_CURRENT_LIMIT / 0.5;

        controlCfgKicker.Slot0.kP = 0.09;
        controlCfgKicker.Slot0.kI = 0;
        controlCfgKicker.Slot0.kD = 0.001;
        controlCfgKicker.Slot0.kV = 0.12; // ~12V

        left.getConfigurator().apply(controlCfg);
        center.getConfigurator().apply(controlCfg);
        right.getConfigurator().apply(controlCfgRight);
        kicker.getConfigurator().apply(controlCfgKicker);

        this.isRed = isRed;

        // calculateShooterCurves();
    }

    // private void calculateShooterCurves() {
    // WeightedObservedPoints leftPoints = new WeightedObservedPoints();
    // for (int i = 0; i < leftShooterValues.length; i++) {
    // double[] values = leftShooterValues[i];

    // leftPoints.add(values[0], values[1]);
    // }

    // // Fit cubic polynomial
    // PolynomialCurveFitter leftFitter = PolynomialCurveFitter.create(3);
    // leftCoeffs = leftFitter.fit(leftPoints.toList());

    // WeightedObservedPoints centerPoints = new WeightedObservedPoints();
    // for (int i = 0; i < centerShooterValues.length; i++) {
    // double[] values = centerShooterValues[i];

    // centerPoints.add(values[0], values[1]);
    // }

    // // Fit cubic polynomial
    // PolynomialCurveFitter centerFitter = PolynomialCurveFitter.create(3);
    // centerCoeffs = centerFitter.fit(centerPoints.toList());

    // WeightedObservedPoints rightPoints = new WeightedObservedPoints();
    // for (int i = 0; i < rightShooterValues.length; i++) {
    // double[] values = rightShooterValues[i];

    // rightPoints.add(values[0], values[1]);
    // }

    // // Fit cubic polynomial
    // PolynomialCurveFitter rightFitter = PolynomialCurveFitter.create(3);
    // rightCoeffs = rightFitter.fit(rightPoints.toList());
    // }

    public boolean isVelocityWithinTolerance() {
        double tolerancePercent = 0.05; // 5%

        boolean leftReady = Math.abs(getTargetRpmLeft() - getSpeedRpmLeft()) <= getTargetRpmLeft() * tolerancePercent;

        boolean rightReady = Math.abs(getTargetRpmRight() - getSpeedRpmRight()) <= getTargetRpmRight()
                * tolerancePercent;

        boolean centerReady = Math.abs(getTargetRpmCenter() - getSpeedRpmCenter()) <= getTargetRpmCenter()
                * tolerancePercent;

        return leftReady && rightReady && centerReady;
    }

    private void stopAll() {
        kicker.stopMotor();
        center.stopMotor();
        left.stopMotor();
        right.stopMotor();
        m_hopper.stop2();
    }

    private boolean autoShooting = false;
    private Command existingAutoShootCommand = new FunctionalCommand(
            () -> {
                rotate(getTargetRpmLeft(), getTargetRpmRight(), getTargetRpmCenter());
            },
            () -> {
                rotate(getTargetRpmLeft(), getTargetRpmRight(), getTargetRpmCenter());
                if (isVelocityWithinTolerance()) {
                    setKickerControl();
                    m_hopper.startRoll();
                }
            },
            interrupted -> {
                stopAll();
            },
            () -> false,
            this);

    public boolean getIsAutoShooting() {
        return autoShooting;
    }

    /**
     * 1.65 meters away
     */
    public void shoot165() {
        targetRpmLeft = 3100;
        targetRpmCenter = 3700;
        targetRpmRight = 3100;
        m_hood.positionCommand(0.2);
    }

    /**
     * 2.5 meters away
     */
    public void shoot250() {
        targetRpmLeft = 3500;
        targetRpmCenter = 4100;
        targetRpmRight = 3500;
        m_hood.positionCommand(0.2);
    }

    /**
     * 3.5 meters away
     */
    public void shoot350() {
        targetRpmLeft = 3650;
        targetRpmCenter = 4400;
        targetRpmRight = 3750;
        m_hood.positionCommand(0.5);
    }

    public void autoShoot() {
        System.out.println("AUTOSHOOTING: " + autoShooting);
        if (autoShooting) {
            autoShooting = false;
            existingAutoShootCommand.cancel();
        } else {
            autoShooting = true;
            CommandScheduler.getInstance().schedule(existingAutoShootCommand);
        }
    }

    public Command runShooterBack() {
        return new FunctionalCommand(
                () -> {
                    rotate(-500, -500, -500);
                },
                () -> {
                    rotate(-500, -500, -500);
                },
                interrupted -> {
                    isVelocityWithinTolerance();
                },
                () -> false,
                this);
    }

    public void setSelected(Side side) {
        selectedSide = side;
    }

    private void setTargetRpmCenter(double rpm) {
        if (rpm > MAX_RPM)
            rpm = MAX_RPM;
        if (rpm < -MAX_RPM)
            rpm = -MAX_RPM;
        targetRpmCenter = rpm;
    }

    private void setTargetRpmLeft(double rpm) {
        if (rpm > MAX_RPM)
            rpm = MAX_RPM;
        if (rpm < -MAX_RPM)
            rpm = -MAX_RPM;
        targetRpmLeft = rpm;
    }

    private void setTargetRpmRight(double rpm) {
        if (rpm > MAX_RPM)
            rpm = MAX_RPM;
        if (rpm < -MAX_RPM)
            rpm = -MAX_RPM;
        targetRpmRight = rpm;
    }

    private void setTargetRpmKicker(double rpm) {
        if (rpm > MAX_RPM)
            rpm = MAX_RPM;
        if (rpm < -MAX_RPM)
            rpm = -MAX_RPM;
        targetRpmKicker = rpm;
    }

    /**
     * sets targetrpm for all three motors
     * 
     * @param rpm
     */
    private void setTargetRpm(double rpm) {
        if (rpm > MAX_RPM)
            rpm = MAX_RPM;
        if (rpm < -MAX_RPM)
            rpm = -MAX_RPM;
        targetRpmRight = rpm;
        targetRpmLeft = rpm;
        targetRpmCenter = rpm;
    }

    private void toggleAutoRange() {
        doAutoShoot = !doAutoShoot;
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

    public double getTargetRpmKicker() {
        return targetRpmKicker;
    }

    public void increaseSelectedTarget(double deltaRpm) {
        if (selectedSide == Side.LEFT) {
            setTargetRpmLeft(targetRpmLeft + deltaRpm);
        } else if (selectedSide == Side.CENTER) {
            setTargetRpmCenter(targetRpmCenter + deltaRpm);
        } else if (selectedSide == Side.RIGHT) {
            setTargetRpmRight(targetRpmRight + deltaRpm);
        } else {
            setTargetRpmKicker(targetRpmKicker + deltaRpm);
        }
    }

    public void decreaseSelectedTarget(double deltaRpm) {
        if (selectedSide == Side.LEFT) {
            setTargetRpmLeft(targetRpmLeft - deltaRpm);
        } else if (selectedSide == Side.CENTER) {
            setTargetRpmCenter(targetRpmCenter - deltaRpm);
        } else if (selectedSide == Side.RIGHT) {
            setTargetRpmRight(targetRpmRight - deltaRpm);
        } else {
            setTargetRpmKicker(targetRpmKicker - deltaRpm);
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

    public void setKickerControl() {
        kicker.setControl(velocityRequest.withVelocity(targetRpmKicker * RPM_TO_RPS));
    }

    public void rotateKicker() {
        if (isKicking) {
            kicker.stopMotor();
            isKicking = false;
        } else {
            setKickerControl();
            isKicking = true;
        }
    }

    /**
     * Runs the motor at the specified RPM using closed-loop control.
     * 
     * @param rpm Target RPM
     */
    public void rotate(double rpmLeft, double rpmRight, double rpmCenter) {
        // Convert RPM to RPS
        left.setControl(velocityRequest.withVelocity(rpmLeft * RPM_TO_RPS));
        center.setControl(velocityRequest.withVelocity(rpmCenter * RPM_TO_RPS));
        right.setControl(velocityRequest.withVelocity(rpmRight * RPM_TO_RPS));
    }

    public void rotateAtCached() {
        if (isShooting) {
            isShooting = false;
            stop();
        } else {
            isShooting = true;
            rotate(targetRpmLeft, targetRpmRight, targetRpmCenter);
        }
    }

    public void stopKicker() {
        kicker.stopMotor();
    }

    public void stop() {
        left.stopMotor();
        right.stopMotor();
        center.stopMotor();
    }

    public Command toggleAutoShoot() {
        return new InstantCommand(() -> toggleAutoRange());
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

    /**
     * @return Current velocity in RPM
     */
    public double getSpeedRpmKicker() {
        return kicker.getRotorVelocity().getValueAsDouble() * 60.0;
    }

    public double getShooterSupplyCurrent() {
        return right.getSupplyCurrent().getValueAsDouble() +
                left.getSupplyCurrent().getValueAsDouble() +
                center.getSupplyCurrent().getValueAsDouble();
    }

    public boolean isRunning() {
        return getShooterSupplyCurrent() > 0;
    }

    public boolean isShooting() {
        return isShooting;
    }

    public boolean isKicking() {
        return isKicking;
    }

    public double getDistToHub() {
        Translation2d absoluteTargetTranslation = getAbsoluteTranslation(isRed);

        double delta_x = absoluteTargetTranslation.getX() - m_drivetrain.getRobotX();
        double delta_y = absoluteTargetTranslation.getY() - m_drivetrain.getRobotY();

        double hyp = Math.sqrt(delta_x * delta_x + delta_y * delta_y);

        return hyp;
    }

    double storedLeftRPM;
    double storedCenterRPM;
    double storedRightRPM;
    boolean isFirstCycleAuto = true;

    @Override
    public void periodic() {
        if (doAutoShoot) {
            if (isFirstCycleAuto) {
                storedLeftRPM = targetRpmLeft;
                storedCenterRPM = targetRpmCenter;
                storedRightRPM = targetRpmRight;

                isFirstCycleAuto = false;
            }

            double dist = getDistToHub();

            // ChassisSpeeds speeds = m_drivetrain.getCurrentRobotChassisSpeeds(); // check
            // if needs to be made into robo TODO
            // // rel, idk what default is

            // double neededVel = Ballistics.CalculateNeededShooterSpeed(hyp,
            // speeds.vxMetersPerSecond,
            // speeds.vyMetersPerSecond);

            // double neededLeftRPM = getValueFromCurve(neededVel, leftCoeffs);
            // double neededCenterRPM = getValueFromCurve(neededVel, centerCoeffs);
            // double neededRightRPM = getValueFromCurve(neededVel, rightCoeffs);

            applyBestSettingsForDistance(dist);
        } else {
            isFirstCycleAuto = true;
        }
    }

    private Translation2d getAbsoluteTranslation(boolean isRed) {
        if (isRed) {
            return new Translation2d(11.915394, 4.034536);
        } else {
            return new Translation2d(4.625594, 4.034536);
        }
    }

    /**
     * Automatically selects the closest calibrated point
     * and applies hood + all RPM targets.
     */
    private void applyBestSettingsForDistance(double distanceMeters) {

        if (values.length == 0) {
            return;
        }

        double[] closestRow = values[0];
        double smallestError = Math.abs(distanceMeters - values[0][0]);

        for (int i = 1; i < values.length; i++) {
            double error = Math.abs(distanceMeters - values[i][0]);

            if (error < smallestError) {
                smallestError = error;
                closestRow = values[i];
            }
        }

        // Apply hood
        m_hood.positionCommand(closestRow[1]);

        // Apply RPM targets
        setTargetRpmLeft(closestRow[2]);
        setTargetRpmCenter(closestRow[3]);
        setTargetRpmRight(closestRow[4]);
        setTargetRpmKicker(closestRow[5]);
    }

    private double getValueFromCurve(double xPoint, double[] coeffs) {
        double total = 0;
        double power = 0;

        for (double d : coeffs) {
            total += d * Math.pow(xPoint, power);

            power++;
        }

        return total;
    }
}