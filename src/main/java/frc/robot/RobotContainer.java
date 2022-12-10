// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;



import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import java.util.HashMap;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * This class is where the bulk of the robot should be  declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Intake m_intake = new Intake();
  private final Lift m_lift = new Lift();

  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_controller_2 = new XboxController(1);
  PathPlannerTrajectory traj = PathPlanner.loadPath("New Path", new PathConstraints(3.75, 2.75));

  JoystickButton intakeButtonforward = new JoystickButton(m_controller_2, 2);
  JoystickButton intakeButtonbackward = new JoystickButton(m_controller_2, 4);
  JoystickButton liftButtonUp = new JoystickButton(m_controller_2, 3);
  JoystickButton liftButtonDown = new JoystickButton(m_controller_2, 1);
  JoystickButton liftBackUp = new JoystickButton(m_controller_2, 5);
  JoystickButton liftBackDown = new JoystickButton(m_controller_2, 7);
  JoystickButton liftFrontUp = new JoystickButton(m_controller_2, 8);
  JoystickButton liftFrontDown = new JoystickButton(m_controller_2, 6);
  //JoystickButton gyro = new JoystickButton(m_controller, 6);
  JoystickButton intakeSlowUp = new JoystickButton(m_controller_2, 9);
  JoystickButton intakeSlowDown = new JoystickButton(m_controller_2, 10);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain./
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> -modifyAxis(m_controller.getLeftY()/2) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getLeftX()/2) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(m_controller::getBackButton)
            // No requirements because we don't need to interrupt anything
            .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
       
    intakeButtonforward.whenPressed(new InstantCommand(m_intake::intake_up));
    intakeButtonforward.whenReleased(new InstantCommand(m_intake::intake_stop));
    intakeButtonbackward.whenPressed(new InstantCommand(m_intake::intake_down));
    intakeButtonbackward.whenReleased(new InstantCommand(m_intake::intake_stop));

    liftButtonUp.whenPressed(new InstantCommand(m_lift::LiftUp));
    liftButtonUp.whenReleased(new InstantCommand(m_lift::LiftStop));
    liftButtonDown.whenPressed(new InstantCommand(m_lift::LiftDown));
    liftButtonDown.whenReleased(new InstantCommand(m_lift::LiftStop));
    liftBackDown.whenPressed(new InstantCommand(m_lift::LiftBackDown));
    liftBackUp.whenPressed(new InstantCommand(m_lift::LiftBackUp));
    liftFrontUp.whenPressed(new InstantCommand(m_lift::LiftFrontUp));
    liftFrontDown.whenPressed(new InstantCommand(m_lift::LiftFrontDown));

    liftBackDown.whenReleased(new InstantCommand(m_lift::LiftStop));
    liftBackUp.whenReleased(new InstantCommand(m_lift::LiftStop));
    liftFrontUp.whenReleased(new InstantCommand(m_lift::LiftStop));
    liftFrontDown.whenReleased(new InstantCommand(m_lift::LiftStop));

    intakeSlowDown.whenPressed(new InstantCommand(m_intake::set_slow_intake_motor_down));
    intakeSlowDown.whenReleased(new InstantCommand(m_intake::intake_stop));

    intakeSlowUp.whenPressed(new InstantCommand(m_intake::set_slow_intake_motor_up));
    intakeSlowUp.whenReleased(new InstantCommand(m_intake::intake_stop));
    


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
      HashMap<String, Command> eventMap = new HashMap<>();
      return new SequentialCommandGroup(
              new InstantCommand(() -> {
                      if(true){
                              m_drivetrainSubsystem.resetOdometry(traj.getInitialHolonomicPose());
                      }
              }),
              new PPSwerveControllerCommand(
                      traj,
                      m_drivetrainSubsystem::getPose,
                      m_drivetrainSubsystem.m_kinematics,
                      new PIDController(0, 0, 0),
                      new PIDController(0, 0, 0),
                      new PIDController(0, 0, 0),
                      (SwerveModuleState[] states) -> {
                              m_drivetrainSubsystem.m_chassisSpeeds = m_drivetrainSubsystem.m_kinematics.toChassisSpeeds(states);
                      },
                      eventMap,
                      m_drivetrainSubsystem
              )
      );
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.08);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
