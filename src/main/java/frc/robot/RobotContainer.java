// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ClimberJoystickCmd;
import frc.robot.commands.MechanumDriveCmd;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.General;
import frc.robot.Constants.MechanumDriveConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final Joystick joystick = new Joystick(General.JOYSTICK_PORT);
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    m_DriveSubsystem.setDefaultCommand(new MechanumDriveCmd(m_DriveSubsystem, 
      () -> -joystick.getRawAxis(MechanumDriveConstants.JOYSTICK_X_SPEED_AXIS), 
      () -> joystick.getRawAxis(MechanumDriveConstants.JOYSTICK_Y_SPEED_AXIS), 
      () -> joystick.getRawAxis(MechanumDriveConstants.JOYSTICK_Z_ROTATION_AXIS)));

    configureBottonBindings();
  }

  private void configureBottonBindings() {
    new JoystickButton(joystick, buttonNumber1).whileTrue(new ClimberJoystickCmd(m_ClimbSubsystem, /*speed degeri girilecek */)); 
    new JoystickButton(joystick, buttonNumber2).whileTrue(new ClimberJoystickCmd(m_ClimbSubsystem, /*speed degerinin negativi girilecek */)); 
  }


  public Command getAutonomousCommand() {
    // Will decide later
  return null;
}
}
