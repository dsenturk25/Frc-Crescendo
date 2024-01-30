// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.IntakeArmCmd;
import frc.robot.commands.MechanumDriveCmd;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.General;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MechanumDriveConstants;
import frc.robot.commands.ClimbCmd;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();


  private final XboxController joystick = new XboxController(General.JOYSTICK_PORT);
;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    m_DriveSubsystem.setDefaultCommand(new MechanumDriveCmd(m_DriveSubsystem, 
      () -> -joystick.getRawAxis(MechanumDriveConstants.JOYSTICK_X_SPEED_AXIS), 
      () -> joystick.getRawAxis(MechanumDriveConstants.JOYSTICK_Y_SPEED_AXIS), 
      () -> joystick.getRawAxis(MechanumDriveConstants.JOYSTICK_Z_ROTATION_AXIS)));

    configureBindings();
  }

  private void configureBindings() {
    
    new JoystickButton(joystick, IntakeConstants.JOYSTICK_ARM_LOW_BUTTON).whileTrue(new IntakeArmCmd(m_IntakeSubsystem, 0));
    new JoystickButton(joystick, IntakeConstants.JOYSTICK_ARM_MEDIUM_BUTTON).whileTrue(new IntakeArmCmd(m_IntakeSubsystem, 1));
    new JoystickButton(joystick, IntakeConstants.JOYSTICK_ARM_HIGH_BUTTON).whileTrue(new IntakeArmCmd(m_IntakeSubsystem, 2));
    new JoystickButton(joystick, ClimbConstants.CLIMB_UP_BUTTON).whileTrue(new ClimbCmd(m_ClimbSubsystem, ClimbConstants.CLIMB_SPEED)); 
    new JoystickButton(joystick, ClimbConstants.CLIMB_DOWN_BUTTON).whileTrue(new ClimbCmd(m_ClimbSubsystem, -ClimbConstants.CLIMB_SPEED)); 
  }


  public Command getAutonomousCommand() {
    return null;
  }
}
