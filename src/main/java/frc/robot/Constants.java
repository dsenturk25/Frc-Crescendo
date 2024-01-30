// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  public static class General {
    public static final int JOYSTICK_PORT = 0;
  }

  public static class MechanumDriveConstants {
    public static final int JOYSTICK_X_SPEED_AXIS = 3;
    public static final int JOYSTICK_Y_SPEED_AXIS = 1;
    public static final int JOYSTICK_Z_ROTATION_AXIS = 4;

    public static final int LEFT_MOTOR_FRONT_PORT = 1;
    public static final int LEFT_MOTOR_REAR_PORT = 2;
    public static final int RIGHT_MOTOR_FRONT_PORT = 3;
    public static final int RIGHT_MOTOR_REAR_PORT = 4;
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
  }

  public static class ClimbConstants {
    public static final int CLIMB_MOTOR_PORT = 7;

    public static final int CLIMB_UP_BUTTON = 6;
    public static final int CLIMB_DOWN_BUTTON = 7;
    public static final double CLIMB_SPEED = 0.5;
  }
}
