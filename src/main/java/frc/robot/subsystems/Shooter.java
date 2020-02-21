/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkMax left, right;
  private CANEncoder leftEncoder, rightEncoder;
  private CANPIDController leftPID, rightPID;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, rpmSetpoint;
  private boolean pidEnabled = false;
  private DoubleSupplier backupSpeed;

  public Shooter(DoubleSupplier _backupSpeed) {
    backupSpeed = _backupSpeed;

    left = new CANSparkMax(Constants.SHOOTER_LEFT, MotorType.kBrushless);
    right = new CANSparkMax(Constants.SHOOTER_RIGHT, MotorType.kBrushless);

    leftEncoder = left.getEncoder();
    rightEncoder = right.getEncoder();

    leftPID = left.getPIDController();
    rightPID = right.getPIDController();

    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    leftPID.setP(kP);
    rightPID.setP(kP);
    leftPID.setI(kI);
    rightPID.setI(kI);
    leftPID.setD(kD);
    rightPID.setD(kD);
    leftPID.setIZone(kIz);
    rightPID.setIZone(kIz);
    leftPID.setFF(kFF);
    rightPID.setFF(kFF);
    leftPID.setOutputRange(kMinOutput, kMaxOutput);
    rightPID.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("kIz", kIz);
    SmartDashboard.putNumber("kFF", kFF);
  }

  @Override
  public void periodic() {
    if (!pidEnabled) {
      rpmSetpoint = backupSpeed.getAsDouble() * maxRPM;
    }

    updatePID();
  }

  public void setRPM(double rpm) {
    rpmSetpoint = rpm;
  }

  public void enablePID() {
    pidEnabled = true;
  }

  public void disablePID() {
    pidEnabled = false;
  }

  public void stop() {
    rpmSetpoint = 0;
  }

  private void updatePID() {
    double p = SmartDashboard.getNumber("kP", kP);
    double i = SmartDashboard.getNumber("kI", kI);
    double d = SmartDashboard.getNumber("kD", kD);
    double iz = SmartDashboard.getNumber("kIz", kIz);
    double ff = SmartDashboard.getNumber("kFF", kFF);

    if (p != kP) {
      kP = p;
      leftPID.setP(p);
      rightPID.setP(p);
    }
    if (i != kI) {
      kI = i;
      leftPID.setI(i);
      rightPID.setI(i);
    }
    if (d != kD) {
      kD = d;
      leftPID.setD(d);
      rightPID.setD(d);
    }
    if (iz != kIz) {
      kIz = iz;
      leftPID.setIZone(iz);
      rightPID.setIZone(iz);
    }
    if (ff != kFF) {
      kFF = ff;
      leftPID.setFF(ff);
      rightPID.setFF(ff);
    }

    leftPID.setReference(rpmSetpoint, ControlType.kVelocity);
    rightPID.setReference(rpmSetpoint, ControlType.kVelocity);

    SmartDashboard.putNumber("RPM Setpoint", rpmSetpoint);
    SmartDashboard.putNumber("Avg. RPM", (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2);
  }
}
