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

  public static final double HOPPER_SPEED = 0.5;
  public static final double CONVEYOR_SPEED = 0.5;
  public static final double PVC_WINCH_SPEED = 0.8;

  // --------------------
  // Vision
  // --------------------

  public static final double VISION_DIST_CENTER = 12 * 20;
  public static final double VISION_RPM_CENTER = 4500;

  // --------------------
  // Ports
  // --------------------

  // PWM
  public static final int DRIVE_LEFT_FRONT = 0;
  public static final int DRIVE_LEFT_BACK = 1;
  public static final int DRIVE_RIGHT_FRONT = 2;
  public static final int DRIVE_RIGHT_BACK = 3;
  public static final int HOPPER_LEFT = 4;
  public static final int HOPPER_RIGHT = 5;
  public static final int CLIMB_PVC_WINCH = 6;
  public static final int CLIMB_WINCH_LEFT = 7;
  public static final int CLIMB_WINCH_RIGHT = 8;
  public static final int CONVEYOR = 9;

  // CAN
  public static final int SHOOTER_LEFT = 0;
  public static final int SHOOTER_RIGHT = 1;
  public static final int INTAKE = 2;

  // USB
  public static final int JOYSTICK_LEFT = 0;
  public static final int JOYSTICK_RIGHT = 1;
  public static final int XBOX = 3;

  // DIO
  public static final int ENCODER_LEFT_1 = 0;
  public static final int ENCODER_LEFT_2 = 1;
  public static final int ENCODER_RIGHT_1 = 2;
  public static final int ENCODER_RIGHT_2 = 3;
}
