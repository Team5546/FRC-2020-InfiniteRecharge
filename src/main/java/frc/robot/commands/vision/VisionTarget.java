/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class VisionTarget extends CommandBase {
  private DriveTrain driveTrain;
  private Hopper hopper;
  private Shooter shooter;
  private double distance, rpm;

  public VisionTarget(DriveTrain _driveTrain, Hopper _hopper, Shooter _shooter, double _distance, double _rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = _driveTrain;
    hopper = _hopper;
    shooter = _shooter;
    distance = _distance;
    rpm = _rpm;
    addRequirements(driveTrain, hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // 20 ft x 12 in
    driveTrain.setVisionSetpoint(distance);
    driveTrain.enableLimelightLED();
    driveTrain.setState(DriveTrain.DriveTrainState.VISION);
    shooter.setRPM(rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveTrain.onTarget()) {
      hopper.driveFeeder(0.5);
      hopper.driveConveyor(0.5);
    } else {
      hopper.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stop();
    driveTrain.setState(DriveTrain.DriveTrainState.TELEOP);
    driveTrain.disableLimelightLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
