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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  public enum DriveTrainState {
    VISION, AUTONOMOUS, AUTONOMOUS_STRAIGHT, AUTONOMOUS_ROTATE, TELEOP, STOP, TEST
  }

  private DriveTrainState state;

  private static final double LIMELIGHT_HEIGHT = 24;
  private static final double VISION_TARGET_HEIGHT = 89.75;
  private static final double LIMELIGHT_ANGLE = 16.5;
  private static final double VISION_TOLERANCE = 1;
  private static final double VISION_SPEED = 0.3;
  private static final double VISION_MIN_SPEED = 0.5;

  private static final double PULLEY_DIAMETER = 3.5;
  private static final double PULSE_PER_REVOLUTION = 2048;
  private static final double STRAIGHT_TOLERANCE = .5;

  private VictorSP leftBack, leftFront, rightBack, rightFront;
  private SpeedControllerGroup left, right;
  private DifferentialDrive drive;

  private ADIS16448_IMU imu;
  private Encoder leftEncoder, rightEncoder;

  private double visionSetpoint = 0;
  private double straightSetpoint = 0;

  private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry limelightTx = limelightTable.getEntry("tx");
  private NetworkTableEntry limelightTy = limelightTable.getEntry("ty");
  private NetworkTableEntry limelightLED = limelightTable.getEntry("ledMode");
  private NetworkTableEntry limelightCamMode = limelightTable.getEntry("camMode");

  private DoubleSupplier leftStickSpeed, rightStickSpeed;
  private double autoLeftSpeed, autoRightSpeed;

  private double initialBatteryVoltage = 0.0;
  private double teleSpeed = 0.8;

  public DriveTrain(DoubleSupplier _leftStickSpeed, DoubleSupplier _rightStickSpeed) {
    leftBack = new VictorSP(Constants.DRIVE_LEFT_BACK);
    leftFront = new VictorSP(Constants.DRIVE_LEFT_FRONT);
    rightBack = new VictorSP(Constants.DRIVE_RIGHT_BACK);
    rightFront = new VictorSP(Constants.DRIVE_RIGHT_FRONT);

    left = new SpeedControllerGroup(leftBack, leftFront);
    right = new SpeedControllerGroup(rightBack, rightFront);

    drive = new DifferentialDrive(left, right);

    imu = new ADIS16448_IMU();
    leftEncoder = new Encoder(Constants.ENCODER_LEFT_A,Constants.ENCODER_LEFT_B);
    rightEncoder = new Encoder(Constants.ENCODER_RIGHT_A, Constants.ENCODER_RIGHT_B);
    leftEncoder.setDistancePerPulse(PULLEY_DIAMETER / PULSE_PER_REVOLUTION);
    rightEncoder.setDistancePerPulse(PULLEY_DIAMETER / PULSE_PER_REVOLUTION);

    leftStickSpeed = _leftStickSpeed;
    rightStickSpeed = _rightStickSpeed;

    state = DriveTrainState.TELEOP;
  }

  @Override
  public void periodic() {
    double tx = limelightTx.getDouble(0.0);
    double ty = limelightTy.getDouble(0.0);

    switch (state) {
    case STOP:
      SmartDashboard.putString("DriveTrain State", "STOP");
      disableLimelightLED();
      left.set(0);
      right.set(0);
      break;
    case VISION:
      enableLimelightLED();
      SmartDashboard.putString("DriveTrain State", "VISION");
      // SmartDashboard.putNumber("Limelight TX", tx);
      // SmartDashboard.putNumber("Limelight TY", ty);
      // SmartDashboard.putNumber("Limelight Distance",
      //     ((VISION_TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(LIMELIGHT_ANGLE + ty))));
      double[] visionOutputs = getVisionOutput();
      // SmartDashboard.putNumber("Rotation", visionOutputs[0]);
      double leftSpeed = Math.abs(tx) > VISION_TOLERANCE ? Math.copySign(VISION_MIN_SPEED, visionOutputs[0]) + (visionOutputs[0] * VISION_SPEED) : 0;
      double rightSpeed = Math.abs(tx) > VISION_TOLERANCE ? Math.copySign(VISION_MIN_SPEED, -visionOutputs[0]) + (visionOutputs[0] * VISION_SPEED) : 0;
      // SmartDashboard.putNumber("Left", leftSpeed);
      // SmartDashboard.putNumber("Right", rightSpeed);
      double voltageAdjust = initialBatteryVoltage < 12.1 ? (initialBatteryVoltage < 11.8 ? 1.8 : 1.5) : 1.0;
      drive.tankDrive(Math.max(-1, Math.min(1, leftSpeed*voltageAdjust)), Math.max(-1, Math.min(1, rightSpeed*voltageAdjust)));
      break;
    case TEST:
      enableLimelightLED();
      SmartDashboard.putString("DriveTrain State", "TEST");
      // SmartDashboard.putNumber("Limelight TX", tx);
      // SmartDashboard.putNumber("Limelight TY", ty);
      // SmartDashboard.putNumber("Limelight Distance", ((VISION_TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(LIMELIGHT_ANGLE + ty))));
    case AUTONOMOUS:
      drive.tankDrive(autoLeftSpeed, autoRightSpeed);
      break;
    case AUTONOMOUS_STRAIGHT:
      double straightOutput = getStraightOutput();
      drive.tankDrive(straightOutput, straightOutput);
      break;
    default:
      disableLimelightLED();
      SmartDashboard.putString("DriveTrain State", "TELEOP");
      drive.tankDrive(leftStickSpeed.getAsDouble() * teleSpeed, rightStickSpeed.getAsDouble() * teleSpeed);
      break;
    }
  }

  public void resetDistance() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  // public void resetRotation() {
  //   imu.reset();
  // }

  public void setState(DriveTrainState _state) {
    state = _state;
  }

  public void setTeleSpeed(double speed) {
    teleSpeed = speed;
  }

  public void setStraightSetpoint(double setpoint) {
    straightSetpoint = setpoint;
  }

  // public void setRotateSetpoint(double setpoint) {
  //   rotateSetpoint = setpoint;
  // }

  public void setAutoSpeed(double leftSpeed, double rightSpeed) {
    autoLeftSpeed = leftSpeed;
    autoRightSpeed = rightSpeed;
  }

  public void setBatteryVoltage(double voltage) {
    initialBatteryVoltage = voltage;
  }

  public void setVisionSetpoint(double distance) {
    visionSetpoint = distance;
  }

  public boolean onTarget() {
    double tx = limelightTx.getDouble(0.0);

    return Math.abs(tx) < VISION_TOLERANCE;
  }

  public boolean isOnStraightTarget() {
    double error = straightSetpoint - ((leftEncoder.getDistance() + rightEncoder.getDistance()) / 2);
    return error < STRAIGHT_TOLERANCE;
  }

  public void enableLimelightLED() {
    limelightLED.setNumber(3);
    limelightCamMode.setNumber(0);
  }

  public void disableLimelightLED() {
    limelightLED.setNumber(1);
    limelightCamMode.setNumber(1);
  }

  private double getStraightOutput() {
    // Error = Setpoint - Avg. of two encoder distances
    double error = straightSetpoint - ((leftEncoder.getDistance() + rightEncoder.getDistance()) / 2);
    double theoreticalPower = 0;
    if (Math.abs(error) < 100) theoreticalPower = Math.sqrt(Math.abs(error)) / 10;
    else theoreticalPower = 1;
    theoreticalPower = Math.copySign(theoreticalPower, error);
    return theoreticalPower * Constants.AUTONOMOUS_STRAIGHT_SPEED;
  }

  // private double getRotateOutput() {
  //   // Error = Setpoint - IMU Angle
  //   double error = rotateSetpoint - imu.getGyroAngleZ();
  //   rotateIntegral += (error * LOOP_TIME);
  //   double derivative = (error - rotatePrevError) / LOOP_TIME;
  //   return Math.max(-rotateMaxOutput,
  //       Math.min(rotateMaxOutput, rotateKp * error + rotateKi * rotateIntegral + rotateKd * derivative));
  // }

  // Returns [distanceOutput, rotateOutput]
  private double[] getVisionOutput() {
    double tx = limelightTx.getDouble(0.0);
    double ty = limelightTy.getDouble(0.0);

    double headingError = tx;
    // Error = Ideal Distance - Distance to Target
    double distanceError = visionSetpoint
         - ((VISION_TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(LIMELIGHT_ANGLE + ty)));
    // double steeringAdjust = 0.0;
    // if (tx > 1.0)
    //   steeringAdjust = rotateKp * headingError - visionMinAimCommand;
    // else if (tx < 1.0)
    //   steeringAdjust = rotateKp * headingError + visionMinAimCommand;
    // double distanceAdjust = distanceKp * distanceError;

    // FOV = 60, so tx can be (-30,30), maps between (-1,1)
    double steeringAdjust = headingError/30;
    double distanceAdjust = Math.max(-1, Math.min(1, distanceError));


    double[] outputs = { steeringAdjust, distanceAdjust };
    SmartDashboard.putNumberArray("Vision Outputs", outputs);
    return outputs;
  }

  public double getDistance() {
    double ty = limelightTy.getDouble(0.0);
    return ((VISION_TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(LIMELIGHT_ANGLE + ty)));
  }
}
