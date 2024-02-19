// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  
  private PhotonCamera m_camera;

  private final CANSparkMax m_leftA = new CANSparkMax(DriveConstants.LeftMotorCAN[0], MotorType.kBrushless);
  private final CANSparkMax m_leftB = new CANSparkMax(DriveConstants.LeftMotorCAN[1], MotorType.kBrushless);
  private final CANSparkMax m_rightA = new CANSparkMax(DriveConstants.RightMotorCAN[0], MotorType.kBrushless);
  private final CANSparkMax m_rightB = new CANSparkMax(DriveConstants.RightMotorCAN[1], MotorType.kBrushless);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftA, m_rightA);

  //Built in encoders
  private final RelativeEncoder m_leftEncoder = m_leftA.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightA.getEncoder();

  private final PigeonIMU m_pigeon = new PigeonIMU(DriveConstants.PigeonCAN);
  //Still need to add the new IMU (IMU yaw --> gyro)


  //Pose estimation stuff, declare objects
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);
  private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
      m_kinematics,
      Rotation2d.fromDegrees(m_pigeon.getYaw()),
      m_leftEncoder.getPosition(),
      m_rightEncoder.getPosition(),
      new Pose2d(),//starting position
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),//state std devs
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))//cam std devs
      );


  public DriveSubsystem(PhotonCamera camera) {
    m_camera = camera;
    //Configure the motor settings
    m_leftB.follow(m_leftA);
    m_rightB.follow(m_leftA);

    m_leftEncoder.setPositionConversionFactor(-1 * DriveConstants.gearRatio * DriveConstants.WheelDiameterMeters * Math.PI / DriveConstants.kEncoderResolution);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.gearRatio * DriveConstants.WheelDiameterMeters * Math.PI / DriveConstants.kEncoderResolution);

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


  }

  @Override
  public void periodic() {
    update(m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    // m_leftEncoderObj.set(m_leftEncoder.getPosition());
    // m_rightEncoderObj.set(m_rightEncoder.getPosition()); FIND PROPER SYNTAX -- Can you set an encoder object to an arbitrary value?

    SmartDashboard.putNumber("Encoder dist", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("yaw", m_pigeon.getYaw());
    SmartDashboard.putNumber("Estimated X", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Estimated Y", m_poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Estimated Rotation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());

  }

  @Override
  public void simulationPeriodic() {

  }

  //method used to update pose estimator based on encoder values or camera results if available
  public void update(double leftDist, double rightDist) {
    m_poseEstimator.update(Rotation2d.fromDegrees(m_pigeon.getYaw()), leftDist, rightDist);

    var res = m_camera.getLatestResult();
    if (res.hasTargets()) {
      var imageCaptureTime = res.getTimestampSeconds();
      var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
      var camPose = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(res.getBestTarget().getFiducialId()).get().transformBy(camToTargetTrans.inverse());
      SmartDashboard.putNumber("cam to target X", camToTargetTrans.getX());
      SmartDashboard.putNumber("cam to target Y", camToTargetTrans.getY());
      SmartDashboard.putNumber("cam to target Z", camToTargetTrans.getZ());

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
