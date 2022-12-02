// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Lift extends SubsystemBase {
  /** Creates a new Intake. */
  JoystickButton liftUp, liftDown;
  TalonSRX lift_motor_1;
  TalonSRX lift_motor_2;
  MotorControllerGroup lift_motor_group;
  public Lift() {
    lift_motor_1 = new TalonSRX(Constants.LIFT_MOTOR_1);
    lift_motor_2 = new TalonSRX(Constants.LIFT_MOTOR_2);
  }

  public void LiftUp(){
    lift_motor_1.set(ControlMode.PercentOutput, 0.5);
    lift_motor_2.set(ControlMode.PercentOutput, 0.5);
  }

  public void LiftDown(){
    lift_motor_1.set(ControlMode.PercentOutput, 0.5);
    lift_motor_2.set(ControlMode.PercentOutput, 0.5);
  }

  public void LiftStop(){
    lift_motor_1.set(ControlMode.PercentOutput, 0);
    lift_motor_2.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {

  }
}
