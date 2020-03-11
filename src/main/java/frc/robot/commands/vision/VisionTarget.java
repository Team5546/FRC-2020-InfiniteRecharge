/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class VisionTarget extends CommandBase {
  private DriveTrain driveTrain;
  private Hopper hopper;
  private Shooter shooter;
  private double distance = 0;
  private boolean onTarget = false;
  // private int rpmOffset = 133;
  private double m = 3.2508;
  private double b = 3750.7;

  private double initialBatteryVoltage = 0.0;

  private static final double RPM_TOLERANCE = 100;

  public VisionTarget(DriveTrain _driveTrain, Hopper _hopper, Shooter _shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = _driveTrain;
    hopper = _hopper;
    shooter = _shooter;
    addRequirements(driveTrain, hopper);

    initialBatteryVoltage = RobotController.getBatteryVoltage();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setBatteryVoltage(initialBatteryVoltage);
    driveTrain.setVisionSetpoint(distance);
    driveTrain.enableLimelightLED();
    driveTrain.setState(DriveTrain.DriveTrainState.VISION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveTrain.onTarget()) {
      onTarget = true;
      // driveTrain.setState(DriveTrain.DriveTrainState.STOP);
    }

    if (onTarget) {
      SmartDashboard.putString("VISION STATE", "ON TARGET");
      shooter.enablePID();
      double rpm = m * driveTrain.getDistance() + b;
      double rpmOffset = rpm * .05;
      double voltageAdjust = initialBatteryVoltage < 12.1
        ? (initialBatteryVoltage < 11.8 ? 1.8 : 1.5)
        : 1.0;
      double finalRpm = (-rpm - rpmOffset) * voltageAdjust;
      shooter.setRPM(finalRpm);
      SmartDashboard.putNumber("RPM ERROR", shooter.getRPM() + rpm);
      if (Math.abs(shooter.getRPM() + rpm) < RPM_TOLERANCE) {
        hopper.driveFeeder(0.5);
        hopper.driveConveyor(0.5);
      }
    } else {
      SmartDashboard.putString("VISION STATE", "OFF TARGET");
      hopper.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.stop();
    shooter.disablePID();
    driveTrain.setState(DriveTrain.DriveTrainState.TELEOP);
    driveTrain.disableLimelightLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
