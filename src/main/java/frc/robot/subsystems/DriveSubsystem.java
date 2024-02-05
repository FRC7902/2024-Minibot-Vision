// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final CANSparkMax m_leftA = new CANSparkMax(DriveConstants.LeftMotorCAN[0], MotorType.kBrushless);
  private final CANSparkMax m_leftB = new CANSparkMax(DriveConstants.LeftMotorCAN[1], MotorType.kBrushless);
  private final CANSparkMax m_rightA = new CANSparkMax(DriveConstants.RightMotorCAN[0], MotorType.kBrushless);
  private final CANSparkMax m_rightB = new CANSparkMax(DriveConstants.RightMotorCAN[1], MotorType.kBrushless);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftA, m_rightA);

  private final RelativeEncoder m_leftEncoder = m_leftA.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightA.getEncoder();

  private final Encoder m_leftEncoderObj = new Encoder(0, 1);
  private final Encoder m_rightEncoderObj = new Encoder(2, 3);

  private final AnalogGyro m_gyro = new AnalogGyro(DriveConstants.GyroCAN);
  private final PigeonIMU m_pigeon = new PigeonIMU(DriveConstants.PigeonCAN);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);


  private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
      m_kinematics,
      Rotation2d.fromDegrees(m_pigeon.getYaw()),
      m_leftEncoder.getPosition(),
      m_rightEncoder.getPosition(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoderObj);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoderObj);

  private final DifferentialDrivetrainSim m_drivetrainSimulator = DifferentialDrivetrainSim
      .createKitbotSim(KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

  private PhotonCamera m_camera;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(PhotonCamera camera) {
    m_leftB.follow(m_leftA);
    m_rightB.follow(m_leftA);

    m_camera = camera;

    m_leftEncoder.setPositionConversionFactor(-1 * DriveConstants.gearRatio * DriveConstants.WheelDiameterMeters
        * Math.PI / DriveConstants.kEncoderResolution);
    m_rightEncoder.setPositionConversionFactor(
        DriveConstants.gearRatio * DriveConstants.WheelDiameterMeters * Math.PI / DriveConstants.kEncoderResolution);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    m_leftA.setInverted(true);
    m_leftB.setInverted(true);
    m_rightA.setInverted(false);
    m_rightB.setInverted(false);

    m_leftA.setSmartCurrentLimit(45);
    m_leftB.setSmartCurrentLimit(45);
    m_rightA.setSmartCurrentLimit(45);
    m_rightB.setSmartCurrentLimit(45);

    // ----------------------------------------------------------------------------------------

  }

  @Override
  public void periodic() {
    update(m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    // m_leftEncoderObj.set(m_leftEncoder.getPosition());
    // m_rightEncoderObj.set(m_rightEncoder.getPosition());

    SmartDashboard.putNumber("Encoder dist", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("yaw", m_pigeon.getYaw());
    SmartDashboard.putNumber("Estimated X", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Estimated Y", m_poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Estimated Rotation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());

  }

  @Override
  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation
    // connect the motors to update the drivetrain
    m_drivetrainSimulator.setInputs(m_leftA.get() * RobotController.getInputVoltage(),
        m_rightA.get() * RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.02);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  public void update(double leftDist, double rightDist) {
    m_poseEstimator.update(Rotation2d.fromDegrees(m_pigeon.getYaw()), leftDist, rightDist);

    var res = m_camera.getLatestResult();
    if (res.hasTargets()) {
      var imageCaptureTime = res.getTimestampSeconds();
      var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
      var camPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(res.getBestTarget().getFiducialId()).get().transformBy(camToTargetTrans.inverse());

      m_poseEstimator.addVisionMeasurement(camPose.toPose2d(), imageCaptureTime);
    }
  }

  public void driveArcade(double xForward, double zRotation) {
    m_drive.arcadeDrive(xForward, zRotation);
  }

  public void driveRaw(double power) {
    m_leftA.set(power);
    m_rightA.set(power);
  }

  public double getHeading() {// -180,180
    return Math.IEEEremainder(m_pigeon.getYaw(), 360);
  }

  public double getPosition() {
    return m_rightEncoder.getPosition();
  }

  public void turn(double power) {
    m_leftA.set(power);
    m_rightA.set(-power);
  }

  public double modAngle(double angle) {
    return Math.IEEEremainder(angle, 360);
  }

}
