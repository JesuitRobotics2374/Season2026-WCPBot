// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;
  private final TalonFX pivotMotor;
  private boolean raised;
  private boolean lowered;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(14, "FastFD"); // CORRECT
    pivotMotor = new TalonFX(13, "FastFD"); // INCORRECT

    pivotMotor.setPosition(0);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    raised = true;
    lowered = false;
  }

  public Command raiseManual() {
    return new InstantCommand(() -> pivotMotor.set(0.4));
  }

  public Command lowerManual() {
    return new InstantCommand(() -> pivotMotor.set(-0.4));
  }

  public Command stopPivot() {
    return new InstantCommand(() -> pivotMotor.set(0));
  }

  //FIX THESE SOMEDAY

  // public Command raise() {
  //   return new FunctionalCommand(
  //       () -> {
  //       },
  //       () -> {
  //         pivotMotor.set(0.5);
  //         raised = pivotMotor.getPosition().getValueAsDouble() >= 0;
  //       },
  //       interrupted -> {
  //         pivotMotor.set(0);
  //       },
  //       () -> raised,
  //       this);
  // }

  // public Command lower() {
  //   return new FunctionalCommand(
  //       () -> {
  //       },
  //       () -> {
  //         pivotMotor.set(-0.5);
  //         lowered = pivotMotor.getPosition().getValueAsDouble() <= -2.25; //SEVERELY INCORRECT
  //       },
  //       interrupted -> {
  //         pivotMotor.set(0);
  //       },
  //       () -> lowered,
  //       this);
  // }

  public Command intake() {
    return new InstantCommand(() -> intakeMotor.set(0.6), this);
  }

  public Command stop() {
    return new InstantCommand(() -> intakeMotor.set(0), this);
  }

  public Command purge() {
    return new InstantCommand(() -> intakeMotor.set(-0.3), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
