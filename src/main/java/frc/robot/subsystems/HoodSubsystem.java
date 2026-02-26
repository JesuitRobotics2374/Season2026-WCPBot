package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    private static final Distance kServoLength = Millimeters.of(100);
    private static final LinearVelocity kMaxServoSpeed = Millimeters.of(50).per(Second);
    private static final double kMinPosition = 0.1;
    private static final double kMaxPosition = 0.7;
    private static final double kPositionTolerance = 0.02;

    private final Servo leftServo;
    private final Servo rightServo;

    private double currentPosition = 0.2;
    private double targetPosition = 0.2;
    private Time lastUpdateTime = Seconds.of(0);

    public HoodSubsystem() {
        leftServo = new Servo(0);
        rightServo = new Servo(2);
        leftServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        setPosition(currentPosition);
        SmartDashboard.putData(this);
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double position) {
        double clampedPosition = MathUtil.clamp(position, kMinPosition, kMaxPosition);
        leftServo.set(clampedPosition);
        rightServo.set(clampedPosition);
        targetPosition = clampedPosition;
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public Command changePosition(double delta) {
        return positionCommand(currentPosition + delta);
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, kPositionTolerance);
    }

    private void updateCurrentPosition() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            //System.out.println("within tol");
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = kMaxServoSpeed.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(kServoLength).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
    }

    public double getCurrentPos() {
        return currentPosition;
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Position", () -> currentPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, value -> setPosition(value));
    }
}