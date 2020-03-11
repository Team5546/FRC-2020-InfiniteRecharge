/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.ResetEncoders;
import frc.robot.commands.drivetrain.Rotate;
import frc.robot.commands.intake.AutoIntakeSpeed;
import frc.robot.commands.intake.SetIntakeSpeed;
import frc.robot.commands.vision.VisionTarget;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoRightTrench extends SequentialCommandGroup {

  // private double SHOOTING_TIME = 4.7;

  public AutoRightTrench(DriveTrain driveTrain, Hopper hopper, Shooter shooter, Intake intake) {
    super(
      new VisionTarget(driveTrain, hopper, shooter).withTimeout(4.7),
      new ResetEncoders(driveTrain),
      new DriveStraight(driveTrain, -24),
      new Rotate(driveTrain, 180),
      new ResetEncoders(driveTrain),
      new AutoIntakeSpeed(intake, -Constants.INTAKE_SPEED),
      new DriveStraight(driveTrain, 100),
      new AutoIntakeSpeed(intake, 0)
    );
  }
}
