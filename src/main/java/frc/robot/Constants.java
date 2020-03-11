/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  // --------------------
  // Speeds
  // --------------------

  // public static final double INTAKE_SPEED = 0.45;
  public static final double INTAKE_SPEED = 0.6;
  public static final double HOPPER_SPEED = 0.5;
  public static final double CONVEYOR_SPEED = 0.5;
  public static final double PVC_WINCH_SPEED = 1.0;
  public static final double DRIVE_SPEED = 0.8;
  public static final double AUTONOMOUS_STRAIGHT_MAX_SPEED = 0.65;
  public static final double AUTONOMOUS_ROTATE_MAX_SPEED = 0.8;

  // --------------------
  // Vision
  // --------------------

  public static final double VISION_DIST_CENTER = 12 * 20;
  public static final double VISION_RPM_CENTER = 4500;

  // --------------------
  // Ports
  // --------------------

  // PWM
  public static final int DRIVE_LEFT_FRONT = 7;
  public static final int DRIVE_LEFT_BACK = 8;
  public static final int DRIVE_RIGHT_FRONT = 1;
  public static final int DRIVE_RIGHT_BACK = 2;
  public static final int HOPPER_LEFT = 5;
  public static final int HOPPER_RIGHT = 4;
  public static final int CLIMB_PVC_WINCH = 3;
  public static final int CLIMB_WINCH_LEFT = 0;
  public static final int CLIMB_WINCH_RIGHT = 9;
  public static final int CONVEYOR = 6;

  // CAN
  public static final int SHOOTER_LEFT = 1;
  public static final int SHOOTER_RIGHT = 2;
  public static final int INTAKE = 3;

  // USB
  public static final int JOYSTICK_LEFT = 0;
  public static final int JOYSTICK_RIGHT = 1;
  public static final int XBOX = 2;

  // DIO
  public static final int ENCODER_LEFT_A = 8;
  public static final int ENCODER_LEFT_B = 9;
  public static final int ENCODER_RIGHT_A = 1;
  public static final int ENCODER_RIGHT_B = 0;
}
