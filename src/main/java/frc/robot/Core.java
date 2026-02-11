// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.utils.Telemetry;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Core {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final DriveSubsystem drivetrain = TunerConstants.createDrivetrain();

    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final HopperSubsystem m_hopper = new HopperSubsystem();

    public Core() {
        configureBindings();
        configureShuffleBoard();
    }

    public void configureShuffleBoard() {

        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

        tab.addDouble("Speed Center", () -> m_shooter.getSpeedRpmCenter()).withPosition(2, 1).withSize(5, 3);
        tab.addDouble("Target Speed Center", () -> m_shooter.getTargetRpmCenter()).withPosition(7, 1).withSize(2, 1);

        tab.addDouble("Speed Right", () -> m_shooter.getSpeedRpmRight()).withPosition(2, 1).withSize(5, 3);
        tab.addDouble("Target Speed Right", () -> m_shooter.getTargetRpmRight()).withPosition(7, 1).withSize(2, 1);

        tab.addDouble("Speed Left", () -> m_shooter.getSpeedRpmLeft()).withPosition(2, 1).withSize(5, 3);
        tab.addDouble("Target Speed Left", () -> m_shooter.getTargetRpmLeft()).withPosition(7, 1).withSize(2, 1);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise
                                                                                           // with negative X (left)
                ));

        m_shooter.setDefaultCommand(
                Commands.run(() -> {
                    double speed = operatorController.getRightY();
                    if (speed > 0.1) { // Added a small deadband
                        m_shooter.raise();
                    } else if (speed < -0.1) {
                        m_shooter.lower();
                    } else {
                        m_shooter.stopActuator();
                    }
                }, m_shooter));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driveController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        ));

        // Reset the field-centric heading on left bumper press.
        driveController.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // SHOOTER

        operatorController.povUp().onTrue(new InstantCommand(() -> m_shooter.increaseTargetRpm(100)));
        operatorController.povDown().onTrue(new InstantCommand(() -> m_shooter.decreaseTargetRpm(100)));

        operatorController.rightBumper().onTrue(new InstantCommand(() -> m_shooter.increaseTargetRpmLeft(100)));
        operatorController.leftBumper().onTrue(new InstantCommand(() -> m_shooter.decreaseTargetRpmLeft(100)));

        operatorController.rightTrigger().onTrue(new InstantCommand(() -> m_shooter.increaseTargetRpmRight(100)));
        operatorController.leftTrigger().onTrue(new InstantCommand(() -> m_shooter.decreaseTargetRpmRight(100)));

        operatorController.povRight().onTrue(new InstantCommand(() -> m_shooter.increaseTargetRpmCenter(100)));
        operatorController.povLeft().onTrue(new InstantCommand(() -> m_shooter.decreaseTargetRpmCenter(100)));

        operatorController.y().onTrue(new InstantCommand(() -> m_shooter.rotateAtCached()));
        operatorController.x().onTrue(new InstantCommand(() -> m_shooter.stop()));

        // INTAKE

        operatorController.a().whileTrue(m_intake.intake()).onFalse(m_intake.stop());

        driveController.povUp().whileTrue(m_intake.raiseManual()).onFalse(m_intake.stopPivot());
        driveController.povDown().whileTrue(m_intake.lowerManual()).onFalse(m_intake.stopPivot());

        // HOPPER

        operatorController.back().onTrue(m_hopper.roll());
        operatorController.leftStick().onTrue(m_hopper.stop());

    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
