// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;


import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  TalonSRX motor_intake_1;
  Joystick temp_controller = new Joystick(1);
  public Intake() {
    motor_intake_1 = new WPI_TalonSRX(Constants.MOTOR_INTAKE);
  }

  public void intake_up(){
    motor_intake_1.set(ControlMode.PercentOutput, 0.6);
  }

  public void intake_down(){
    motor_intake_1.set(ControlMode.PercentOutput, -0.6);
  }

  public void intake_stop(){
    motor_intake_1.set(ControlMode.PercentOutput, 0);
  }
  
  public void set_slow_intake_motor_down()
  {
    motor_intake_1.set(ControlMode.PercentOutput, 0.2);
  }
  public void set_slow_intake_motor_up()
  {
    motor_intake_1.set(ControlMode.PercentOutput, -0.2);
  }
 

  @Override
  public void periodic() {}
}
