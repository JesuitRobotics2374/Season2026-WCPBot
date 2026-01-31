// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(12, "FastFD"); //ID not correct

  }

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
