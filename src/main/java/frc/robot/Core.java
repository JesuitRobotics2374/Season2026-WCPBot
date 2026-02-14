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
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Side;

public class Core {
    private double MaxSpeed = 0.4 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = 0.65 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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
    private final HopperSubsystem m_hopper = new HopperSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem(m_hopper);
    private final ClimberSubsystem m_climber = new ClimberSubsystem();
    private final HoodSubsystem m_hood = new HoodSubsystem();

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

        tab.addDouble("Speed Kicker", () -> m_shooter.getSpeedRpmKicker()).withPosition(2, 1).withSize(5, 3);
        tab.addDouble("Target Speed Kicker", () -> m_shooter.getTargetRpmKicker()).withPosition(7, 1).withSize(2, 1);

        tab.addDouble("Climber Rotations", () -> m_climber.getRotations());

        tab.addBoolean("Shooting", () -> m_shooter.isShooting());
        tab.addBoolean("Kicking", () -> m_shooter.isKicking());
        tab.addBoolean("Rolling", () -> m_hopper.isRolling());
        tab.addBoolean("Intaking", () -> m_intake.isIntaking());
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveController.getLeftY() * MaxSpeed * getGlobalSlowMode()) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed * getGlobalSlowMode()) // Drive left with negative X (left)
                        .withRotationalRate(-driveController.getRightX() * MaxAngularRate * getGlobalSlowMode()) // Drive counterclockwise
                                                                                           // with negative X (left)
                ));

        m_hood.setDefaultCommand(
                Commands.run(() -> {
                    double speed = operatorController.getRightY();
                    if (speed > 0.1) { // Added a small deadband
                        m_hood.setPosition(1);
                    } else if (speed < -0.1) {
                        m_hood.setPosition(0);
                    } 
                    else {
                        m_hood.setPosition(m_hood.getPosition());
                    }
                }, m_hood));

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

        operatorController.rightBumper().onTrue(new InstantCommand(() -> m_shooter.increaseTargetRpm(100)));
        operatorController.leftBumper().onTrue(new InstantCommand(() -> m_shooter.decreaseTargetRpm(100)));

        operatorController.rightTrigger().onTrue(new InstantCommand(() -> m_shooter.increaseSelectedTarget(100)));
        operatorController.leftTrigger().onTrue(new InstantCommand(() -> m_shooter.decreaseSelectedTarget(100)));

        operatorController.povRight().onTrue(new InstantCommand(() -> m_shooter.setSelected(Side.RIGHT)));
        operatorController.povUp().onTrue(new InstantCommand(() -> m_shooter.setSelected(Side.CENTER)));
        operatorController.povLeft().onTrue(new InstantCommand(() -> m_shooter.setSelected(Side.LEFT)));
        operatorController.povDown().onTrue(new InstantCommand(() -> m_shooter.setSelected(Side.KICKER)));

        operatorController.y().onTrue(new InstantCommand(() -> m_shooter.rotateAtCached()));

        operatorController.x().onTrue(new InstantCommand(() -> m_shooter.rotateKicker()));

        // INTAKE

        operatorController.a().onTrue(new InstantCommand(() -> m_intake.intake()));

        driveController.povUp().whileTrue(m_intake.raiseManual()).onFalse(m_intake.stopPivot());
        driveController.povDown().whileTrue(m_intake.lowerManual()).onFalse(m_intake.stopPivot());

        driveController.rightBumper().onTrue(m_climber.extendArm());
        driveController.leftBumper().onTrue(m_climber.retractArm());

        driveController.x().whileTrue(m_climber.rotateCommand(-0.3));
        driveController.y().whileTrue(m_climber.rotateCommand(0.3));
        

        // HOPPER

        operatorController.b().onTrue(new InstantCommand(() -> m_hopper.roll()));

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

    private double getGlobalSlowMode() {
        return 1 - 0.75 * driveController.getRightTriggerAxis();
    }
}