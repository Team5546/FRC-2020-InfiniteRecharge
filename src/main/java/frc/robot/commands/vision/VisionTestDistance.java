/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveTrain.DriveTrainState;

public class VisionTestDistance extends CommandBase {
  private DriveTrain driveTrain;
  private Shooter shooter;
  
  public VisionTestDistance(DriveTrain _driveTrain, Shooter _shooter) {
    driveTrain = _driveTrain;
    shooter = _shooter;
    addRequirements(driveTrain, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.enableLimelightLED();
    driveTrain.setState(DriveTrainState.VISION);
    // shooter.enablePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooter.setRPM(SmartDashboard.getNumber("RPM Setpoint", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.disableLimelightLED();
    driveTrain.setState(DriveTrainState.TELEOP);
    shooter.disablePID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
