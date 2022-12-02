// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  TalonSRX motor_intake_1;
  public Intake() {
    motor_intake_1 = new WPI_TalonSRX(Constants.MOTOR_INTAKE);
  }

  public void intake_up(){
    motor_intake_1.set(ControlMode.PercentOutput, 0.5);
  }

  public void intake_down(){
    motor_intake_1.set(ControlMode.PercentOutput, 0.5);
  }

  public void intake_stop(){
    motor_intake_1.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
  }
}
