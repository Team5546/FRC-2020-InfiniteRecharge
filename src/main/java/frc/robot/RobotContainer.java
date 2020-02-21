/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.hopper.Feed;
import frc.robot.commands.vision.VisionTarget;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  private final Joystick leftStick = new Joystick(Constants.JOYSTICK_LEFT);
  private final Joystick rightStick = new Joystick(Constants.JOYSTICK_RIGHT);
  private final XboxController xbox = new XboxController(Constants.XBOX);
  private final JoystickButton hopperFeed = new JoystickButton(xbox, Button.kA.value);
  private final JoystickButton visionTargetCenter = new JoystickButton(xbox, Button.kB.value);

  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain(() -> leftStick.getY(), () -> rightStick.getY());
  private final Hopper hopper = new Hopper();
  private final Shooter shooter = new Shooter(() -> xbox.getTriggerAxis(Hand.kRight));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    hopperFeed.whenHeld(new Feed(hopper));
    visionTargetCenter.whenHeld(
        new VisionTarget(driveTrain, hopper, shooter, Constants.VISION_DIST_CENTER, Constants.VISION_RPM_CENTER));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
