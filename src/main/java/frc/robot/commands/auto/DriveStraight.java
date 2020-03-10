/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveStraight extends CommandBase {
  private DriveTrain driveTrain;
  private double distance;

  public DriveStraight(DriveTrain _driveTrain, double _distance) {
    driveTrain = _driveTrain;
    distance = _distance;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetDistance();
    driveTrain.resetRotation();
    driveTrain.setStraightSetpoint(distance);
    driveTrain.setState(DriveTrain.DriveTrainState.AUTONOMOUS_STRAIGHT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setState(DriveTrain.DriveTrainState.TELEOP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTrain.isOnStraightTarget();
  }
}
