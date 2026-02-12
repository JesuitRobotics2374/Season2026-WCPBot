// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;

public class LinearActuator extends Servo {
  
  private double m_speed;
  private double m_length;
  private double setPos;
  private double curPos;

  double lastTime;

  /**
   * Parameters for L16-R Actuonix Linear Actuators
   *
   * @param channel PWM channel used to control the servo
   * @param length  max length of the servo [mm]
   * @param speed   max speed of the servo [mm/second]
   */
  public LinearActuator(int channel, int length, int speed) {
    super(channel);
    setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    m_length = length;
    m_speed = speed;
    lastTime = Timer.getFPGATimestamp();
  }

  /**
   * Run this method in any periodic function to update the position estimation of
   * your
   * servo
   *
   * @param setpoint the target position of the servo [mm]
   */
  public void setPosition(double setpoint) {
    setPos = MathUtil.clamp(setpoint, 0, m_length);
    setSpeed((setPos / m_length * 2) - 1);
  }

  /**
   * Run this method in any periodic function to update the position estimation of
   * your
   * servo
   */
  public void updateCurPos() {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastTime;
    if (curPos > setPos + m_speed * dt) {
      curPos -= m_speed * dt;
    } else if (curPos < setPos - m_speed * dt) {
      curPos += m_speed * dt;
    } else {
      curPos = setPos;
    }
    lastTime = currentTime;
  }

  /**
   * Current position of the servo, must be calling {@link #updateCurPos()
   * updateCurPos()} periodically
   *
   * @return Servo Position [mm]
   */
  @Override
  public double getPosition() {
    return curPos;
  }

  /**
   * Checks if the servo is at its target position, must be calling
   * {@link #updateCurPos()
   * updateCurPos()} periodically
   * 
   * @return true when servo is at its target
   */
  public boolean isFinished() {
    return Math.abs(curPos - setPos) < 0.5;
  }

}