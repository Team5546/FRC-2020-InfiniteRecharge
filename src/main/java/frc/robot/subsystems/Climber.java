/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private VictorSP pvcWinch, winchLeft, winchRight;
  private XboxController xbox;

  public Climber(XboxController _xbox) {
    pvcWinch = new VictorSP(Constants.CLIMB_PVC_WINCH);
    winchLeft = new VictorSP(Constants.CLIMB_WINCH_LEFT);
    winchRight = new VictorSP(Constants.CLIMB_WINCH_RIGHT);
    winchRight.setInverted(true);

    xbox = _xbox;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setPvcWinchSpeed(xbox.getTriggerAxis(Hand.kLeft));
    setWinchSpeed(xbox.getTriggerAxis(Hand.kRight));
  }

  public void setPvcWinchSpeed(double speed) {
    pvcWinch.set(speed);
  }

  public void stopPvcWinch() {
    pvcWinch.set(0);
  }

  public void setWinchSpeed(double speed) {
    winchLeft.set(speed);
    winchRight.set(speed);
  }

  public void stopWinch() {
    winchLeft.set(0);
    winchRight.set(0);
  }
}
