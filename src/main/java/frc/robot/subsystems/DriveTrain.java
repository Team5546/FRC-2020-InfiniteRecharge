/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  public enum DriveTrainState {
    VISION, AUTONOMOUS_DISTANCE, AUTONOMOUS_ROTATE, TELEOP, STOP
  }

  private DriveTrainState state;

  private static final double LOOP_TIME = 0.2;
  private static final double WHEEL_DIAMETER = 2.85;
  private static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * (WHEEL_DIAMETER / 2);
  private static final double PULSES_PER_REV = 2048;
  private static final double DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / PULSES_PER_REV;
  private static final double LIMELIGHT_HEIGHT = 25;
  private static final double VISION_TARGET_HEIGHT = 89;
  private static final double LIMELIGHT_ANGLE = 15;
  private static final double VISION_TOLERANCE = 1;

  private VictorSP leftBack, leftFront, rightBack, rightFront;
  private SpeedControllerGroup left, right;
  private DifferentialDrive drive;

  private Encoder leftEncoder, rightEncoder;
  private ADIS16448_IMU imu;

  private double distanceKp = 1;
  private double distanceKi = 0;
  private double distanceKd = 0;
  private double distanceIntegral, distancePrevError, distanceSetpoint = 0;
  private double distanceMaxOutput = 0.8;

  private double rotateKp = 1;
  private double rotateKi = 0;
  private double rotateKd = 0;
  private double rotateIntegral, rotatePrevError, rotateSetpoint = 0;
  private double rotateMaxOutput = 0.5;

  private double visionSetpoint = 0;
  private double visionMinAimCommand = 0.05;

  private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry limelightTx = limelightTable.getEntry("tx");
  private NetworkTableEntry limelightTy = limelightTable.getEntry("ty");
  private NetworkTableEntry limelightLED = limelightTable.getEntry("ledMode");

  private DoubleSupplier leftStickSpeed, rightStickSpeed;

  public DriveTrain(DoubleSupplier _leftStickSpeed, DoubleSupplier _rightStickSpeed) {
    leftBack = new VictorSP(Constants.DRIVE_LEFT_BACK);
    leftFront = new VictorSP(Constants.DRIVE_LEFT_FRONT);
    rightBack = new VictorSP(Constants.DRIVE_RIGHT_BACK);
    rightFront = new VictorSP(Constants.DRIVE_RIGHT_FRONT);

    left = new SpeedControllerGroup(leftBack, leftFront);
    right = new SpeedControllerGroup(rightBack, rightFront);

    drive = new DifferentialDrive(left, right);

    leftEncoder = new Encoder(Constants.ENCODER_LEFT_1, Constants.ENCODER_LEFT_2);
    leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    rightEncoder = new Encoder(Constants.ENCODER_RIGHT_1, Constants.ENCODER_RIGHT_2);
    leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    imu = new ADIS16448_IMU();

    leftStickSpeed = _leftStickSpeed;
    rightStickSpeed = _rightStickSpeed;

    state = DriveTrainState.TELEOP;
  }

  @Override
  public void periodic() {
    switch (state) {
    case STOP:
      left.set(0);
      right.set(0);
      break;
    case VISION:
      double[] visionOutputs = getVisionOutput();
      drive.tankDrive(visionOutputs[0] - visionOutputs[1], visionOutputs[0] + visionOutputs[1]);
      break;
    case AUTONOMOUS_DISTANCE:
      double distanceOutput = getDistanceOutput();
      drive.tankDrive(distanceOutput, distanceOutput);
      break;
    case AUTONOMOUS_ROTATE:
      double rotateOutput = getRotateOutput();
      drive.tankDrive(-rotateOutput, rotateOutput);
      break;
    default:
      drive.tankDrive(leftStickSpeed.getAsDouble(), rightStickSpeed.getAsDouble());
      break;
    }
  }

  public void resetDistance() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void resetRotation() {
    imu.reset();
  }

  public void setState(DriveTrainState _state) {
    state = _state;
  }

  public void setDistanceSetpoint(double setpoint) {
    distanceSetpoint = setpoint;
  }

  public void setRotateSetpoint(double setpoint) {
    rotateSetpoint = setpoint;
  }

  public void setVisionSetpoint(double distance) {
    visionSetpoint = distance;
  }

  public boolean onTarget() {
    double tx = limelightTx.getDouble(0.0);
    double ty = limelightTy.getDouble(0.0);

    double distanceError = visionSetpoint
        - ((VISION_TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_ANGLE + ty));

    return Math.abs(tx) < VISION_TOLERANCE && Math.abs(distanceError) < VISION_TOLERANCE;
  }

  public void enableLimelightLED() {
    limelightLED.setNumber(3);
  }

  public void disableLimelightLED() {
    limelightLED.setNumber(1);
  }

  private double getDistanceOutput() {
    // Error = Setpoint - Avg. of two encoder distances
    double error = distanceSetpoint - ((leftEncoder.getDistance() + -rightEncoder.getDistance()) / 2);
    distanceIntegral += (error * LOOP_TIME);
    double derivative = (error - distancePrevError) / LOOP_TIME;
    return Math.max(-distanceMaxOutput,
        Math.min(distanceMaxOutput, distanceKp * error + distanceKi * distanceIntegral + distanceKd * derivative));
  }

  private double getRotateOutput() {
    // Error = Setpoint - IMU Angle
    double error = rotateSetpoint - imu.getGyroAngleZ();
    rotateIntegral += (error * LOOP_TIME);
    double derivative = (error - rotatePrevError) / LOOP_TIME;
    return Math.max(-rotateMaxOutput,
        Math.min(rotateMaxOutput, rotateKp * error + rotateKi * rotateIntegral + rotateKd * derivative));
  }

  // Returns [distanceOutput, rotateOutput]
  private double[] getVisionOutput() {
    double tx = limelightTx.getDouble(0.0);
    double ty = limelightTy.getDouble(0.0);

    double headingError = -tx;
    // Error = Ideal Distance - Distance to Target
    double distanceError = visionSetpoint
        - ((VISION_TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(LIMELIGHT_ANGLE + ty));
    double steeringAdjust = 0.0;
    if (tx > 1.0)
      steeringAdjust = rotateKp * headingError - visionMinAimCommand;
    else if (tx < 1.0)
      steeringAdjust = rotateKp * headingError + visionMinAimCommand;
    double distanceAdjust = distanceKp * distanceError;

    double[] outputs = { steeringAdjust, distanceAdjust };
    return outputs;
  }
}
