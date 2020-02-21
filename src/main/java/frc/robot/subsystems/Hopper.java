/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  private VictorSP left, right, conveyor;

  public Hopper() {
    left = new VictorSP(Constants.HOPPER_LEFT);
    right = new VictorSP(Constants.HOPPER_RIGHT);
    conveyor = new VictorSP(Constants.CONVEYOR);
  }

  @Override
  public void periodic() {
  }

  public void driveFeeder(double speed) {
    left.set(speed);
    right.set(speed);
  }

  public void driveConveyor(double speed) {
    conveyor.set(speed);
  }

  public void unjam(double speed) {
    left.set(-speed);
    right.set(speed);
  }

  public void stop() {
    left.set(0);
    right.set(0);
    conveyor.set(0);
  }
}
