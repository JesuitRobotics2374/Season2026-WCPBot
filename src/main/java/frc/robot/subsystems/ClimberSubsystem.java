// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubsystem extends SubsystemBase {

  private TalonFX climberMotor;
  private TalonFX follower;
  private final double minRotations = 0;
  private final double maxRotations = -31;

  /** Creates a new Climber. */
  public ClimberSubsystem() {

    this.climberMotor = new TalonFX(41);

    follower = new TalonFX(42);

    climberMotor.setPosition(0);
    follower.setPosition(0);
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
    follower.setNeutralMode(NeutralModeValue.Brake);

    follower.setControl(new Follower(climberMotor.getDeviceID(), MotorAlignmentValue.Aligned));

  }
  
  /**
   * Sets the speed of the motor to extend or retract the arm
   * @param speed
   */
  public void setMotorSpeed(double speed) {
    climberMotor.set(speed);
  }

  public Command rotateCommand(double speed) {
    //return new InstantCommand(() -> setClimberMotorSpeed(1));

    return new FunctionalCommand(
      //init
      () -> {setMotorSpeed(speed);},
      //execute
      () -> {},
      //interrupt
      interrupted -> {setMotorSpeed(0);},
      //isFinished
      () -> false,
      //requirements
      this
    );

  }

  /**
   * Extends the arm
   * @return Functional Command to extend arm.
   */
  public Command extendArm() {
    //return new InstantCommand(() -> setClimberMotorSpeed(1));

    return new FunctionalCommand(
      //init
      () -> {setMotorSpeed(0.5);},
      //execute
      () -> {},
      //interrupt
      interrupted -> {setMotorSpeed(0);},
      //isFinished
      () -> hasReachedMax(),
      //requirements
      this
    );

  }

  /**
   * Retracts the arm
   * @return Instant Command to retract arm.
   */
  public Command retractArm() {
    // new InstantCommand(() -> setClimberMotorSpeed(-1));
    return new FunctionalCommand(
      //init
      () -> {setMotorSpeed(-0.5);},
      //execute
      () -> {},
      //interrupt
      interrupted -> {setMotorSpeed(0);},
      //isFinished
      () -> hasReachedMin(),
      //requirements
      this
    );

  }

  public double getRotations() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  /**
   * Checks the position of the motor and stops it
   * @return boolean
   */
  public boolean hasReachedMax() {
    if (climberMotor.getPosition().getValueAsDouble() <= maxRotations) {
      return true;
    }
    return false;
  }

  /**
   * Checks the position of the motor and stops it
   * @return
   */
  public boolean hasReachedMin() {
    if (climberMotor.getPosition().getValueAsDouble() >= minRotations) {
      return true;
    }
    return false;
  }

  public double getClimberSupplyCurrent() {
    return climberMotor.getSupplyCurrent().getValueAsDouble() +
           follower.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}