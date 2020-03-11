/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.AutoCenter;
import frc.robot.commands.auto.AutoRightTrench;
import frc.robot.commands.auto.DriveStraight;
import frc.robot.commands.climber.RaiseClimber;
import frc.robot.commands.drivetrain.Rotate;
import frc.robot.commands.drivetrain.Turbo;
import frc.robot.commands.hopper.Feed;
import frc.robot.commands.hopper.Unjam;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.vision.VisionTarget;
import frc.robot.commands.vision.VisionTarget2;
import frc.robot.commands.vision.VisionTestDistance;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  // Controllers
  private final Joystick leftStick = new Joystick(Constants.JOYSTICK_LEFT);
  private final Joystick rightStick = new Joystick(Constants.JOYSTICK_RIGHT);
  private final XboxController xbox = new XboxController(Constants.XBOX);
  private final JoystickButton hopperFeed = new JoystickButton(xbox, Button.kA.value);
  private final JoystickButton hopperUnjam = new JoystickButton(xbox, Button.kY.value);
  private final JoystickButton visionTargetCenter = new JoystickButton(xbox, Button.kB.value);
  private final JoystickButton intakeSuck = new JoystickButton(leftStick, 1);
  private final JoystickButton intakeShoot = new JoystickButton(rightStick, 1);
  private final JoystickButton vision = new JoystickButton(xbox, Button.kX.value);
  private final JoystickButton turbo1 = new JoystickButton(rightStick, 3);
  private final JoystickButton turbo2 = new JoystickButton(rightStick, 4);

  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain(
    () -> -leftStick.getY(),
    () -> -rightStick.getY()
  );
  private final Hopper hopper = new Hopper();
  private final Shooter shooter = new Shooter(() -> xbox.getY(Hand.kRight));
  private final Intake intake = new Intake();
  private final Climber climber = new Climber(xbox);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    hopperFeed.whenHeld(new Feed(hopper));
    hopperUnjam.whenHeld(new Unjam(hopper));
    visionTargetCenter.whenHeld(new VisionTarget2(driveTrain, hopper, shooter, 200));
    //     new VisionTarget(driveTrain, hopper, shooter, Constants.VISION_DIST_CENTER, Constants.VISION_RPM_CENTER));
    intakeSuck.whenHeld(new SetIntakeSpeed(intake, Constants.INTAKE_SPEED));
    intakeShoot.whenHeld(new SetIntakeSpeed(intake, -Constants.INTAKE_SPEED));
    vision.whenHeld(new VisionTarget(driveTrain, hopper, shooter));
    turbo1.whenHeld(new Turbo(driveTrain));
    turbo2.whenHeld(new Turbo(driveTrain));
  }

  public Command getAutonomousCommand() {
    // return new AutoCenter(driveTrain, hopper, shooter);
    // return new Rotate(driveTrain, 90);
    // return new DriveStraight(driveTrain, 121);
    return new AutoRightTrench(driveTrain, hopper, shooter, intake);
  }

  public void resetDriveTrain() {
    driveTrain.setState(DriveTrain.DriveTrainState.TELEOP);
  }

  public void runDebug() {
    SmartDashboard.putNumber("Left Dist", driveTrain.getEncoderOutput(0));
    SmartDashboard.putNumber("Right Dist", driveTrain.getEncoderOutput(1));
  }
}
