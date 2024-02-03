// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;


public final class Constants {

  public static class General {
    public static final int JOYSTICK_PORT = 0;
  }

  public static class MechanumDriveConstants {
    public static final int JOYSTICK_X_SPEED_AXIS = 1;
    public static final int JOYSTICK_Y_SPEED_AXIS = 2;
    public static final int JOYSTICK_Z_ROTATION_AXIS = 4;

    public static final int LEFT_MOTOR_FRONT_PORT = 0;
    public static final int LEFT_MOTOR_REAR_PORT = 1;
    public static final int RIGHT_MOTOR_FRONT_PORT = 13;
    public static final int RIGHT_MOTOR_REAR_PORT = 14;

    public static final double MECHANUM_WHEEL_LOCATION = 0.381;  // in meters

    public static final double SPEED_THRESHOLD = 0.75;
  }

  public static class ShooterConstants {
    public static final int SHOOT_MOTOR_PORT = 0; //not true value

    public static final int SHOOT_LOW_BUTTON = 1; //not true value
    public static final int SHOOT_HIGH_BUTTON = 2; //not true value
    public static final int HIGH_SHOOT_SPEED = 3; //not true value
    public static final int LOW_SHOOT_SPEED = 1; //not true value
  }

  public static class IntakeConstants {
    public static final double INTAKE_DEGREE = 10.0;
    public static final double LOWER_THROW_DEGREE = 30.0;
    public static final double CANON_FEEDER_DEGREE = 100.0;

    public static final double kP = 0.1;
    public static final double kI = 1;
    public static final double kD = 0.1;

    public static final double kI_LIMIT = 5;

    public static final int ARM_MOTOR_PORT = 5;

    public static final int ENCODER_SOURCE_A = 5;
    public static final int ENCODER_SOURCE_B = 6;

    public static final double K_ARM_TICK_2_DEG = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;

    public static final int JOYSTICK_ARM_LOW_BUTTON = 4;
    public static final int JOYSTICK_ARM_MEDIUM_BUTTON = 5;
    public static final int JOYSTICK_ARM_HIGH_BUTTON = 6;

    public static final int INTAKE_LEFTMOTOR_PORT = 5;
    public static final int INTAKE_RIGHTMOTOR_PORT = 5;
    public static final double INTAKE_MOTOR_SPEED = 0.5;

    public static final int INTAKE_JOYSTICK_PORT = 7;
  }

  public static class ClimbConstants {
    public static final int CLIMB_MOTOR_PORT = 7;

    public static final int CLIMB_UP_BUTTON = 6;
    public static final int CLIMB_DOWN_BUTTON = 7;
    public static final double CLIMB_SPEED = 0.5;
  }

  public static class AutonomousConstants {

    public static final double APRIL_1_X = 0;
    public static final double APRIL_1_Y = 0;

    public static final double APRIL_2_X = 0;
    public static final double APRIL_2_Y = 0;

    public static final double APRIL_Z = 0;

    public static final double DRIVE_FORWARD_SPEED = 0.5;
    public static final double DRIVE_FORWARD_TIME = 5.0;

    public static final double TURN_SPEED = 0.2;

    public static final Transform3d CAM_TO_ROBOT_CENTER = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 3));

    public static final String APRIL_CAMERA_NAME = "";
    public static final String OBJECT_CAMERA_NAME = "";

    public static final double INTAKE_DRIVE_FORWARD_SPEED = 0.5;

    public static final double INTAKE_SPEED = 0.5;
    public static final double INTAKE_TIME = 1;  // second

    public static final double MAX_CONTOUR_AREA_PERCENTAGE = 90.0;

    public static final double kP_DRIVE = 0;
    public static final double kI_DRIVE = 0;
    public static final double kD_DRIVE = 0;

    public static final double kP_TURN = 0;
    public static final double kI_TURN = 0;
    public static final double kD_TURN = 0;
  }
}
