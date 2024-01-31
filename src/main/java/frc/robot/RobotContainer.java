// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.IntakeArmCmd;
import frc.robot.commands.MechanumDriveCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.TurnAroundCmd;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.General;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MechanumDriveConstants;
import frc.robot.commands.AlignAprilCmd;
import frc.robot.commands.AlignObjectCmd;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.DriveForwardCmd;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  private final XboxController joystick = new XboxController(General.JOYSTICK_PORT);
  private final PhotonCamera aprilPhotonCamera = new PhotonCamera(AutonomousConstants.APRIL_CAMERA_NAME);
  // private final PhotonCamera objectPhotonCamera = new PhotonCamera(AutonomousConstants.OBJECT_CAMERA_NAME);

  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  private final PoseSubsystem m_PoseSubsystem = new PoseSubsystem(aprilPhotonCamera, m_DriveSubsystem);
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

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
    return new SequentialCommandGroup(
      new DriveForwardCmd(m_DriveSubsystem, AutonomousConstants.DRIVE_FORWARD_SPEED, AutonomousConstants.DRIVE_FORWARD_TIME),
      new TurnAroundCmd(m_DriveSubsystem, AutonomousConstants.TURN_SPEED),  // will configure to work with Gyro
      new AlignObjectCmd(m_DriveSubsystem, aprilPhotonCamera),
      /* Approach Intake Command - Doğu */
      new AlignAprilCmd(m_PoseSubsystem),
      new IntakeArmCmd(m_IntakeSubsystem, 2),
      new ShootCmd(m_ShooterSubsystem, 0)
    );
  }
}
