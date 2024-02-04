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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);
  private final Pose3d m_objectInField;

  private final Transform3d m_robotToCamera = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, Math.PI));
  
  private final DoubleArrayEntry m_cameraToObjectEntry;

  private final double[] m_defaultVal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  private final Field2d m_fieldSim = new Field2d();
  private final Field2d m_fieldApproximation = new Field2d();

  private final DifferentialDrivePoseEstimator m_poseEstimator = 
    new DifferentialDrivePoseEstimator(
      m_kinematics, 
      Rotation2d.fromDegrees(m_pigeon.getYaw()), 
      m_leftEncoder.getPosition(),
      m_rightEncoder.getPosition(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
      );

  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoderObj);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoderObj);
  private final LinearSystem <N2, N2, N2> m_drivetrainSystem = 
    LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  private final DifferentialDrivetrainSim m_drivetrainSimulator = 
    new DifferentialDrivetrainSim(
      m_drivetrainSystem,
      DCMotor.getCIM(2),
      8,
      DriveConstants.kTrackWidth,
      DriveConstants.kWheelRadius,
      null
    );

  private PhotonCamera m_camera;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(PhotonCamera camera, DoubleArrayTopic cameraToObjectTopic) {
    m_leftB.follow(m_leftA);
    m_rightB.follow(m_leftA);

    m_camera = camera;

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

    m_cameraToObjectEntry = cameraToObjectTopic.getEntry(m_defaultVal);

    m_objectInField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(0).get();

    SmartDashboard.putData("Field", m_fieldSim);
    SmartDashboard.putData("FieldEstimation", m_fieldApproximation);
        
//----------------------------------------------------------------------------------------

  }

  @Override
  public void periodic() {

    //m_leftEncoderObj.set(m_leftEncoder.getPosition());
    //m_rightEncoderObj.set(m_rightEncoder.getPosition());


    SmartDashboard.putNumber("Encoder dist", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("yaw", m_pigeon.getYaw());

  }

  @Override
  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation
    //connect the motors to update the drivetrain
    
  }

  public void publishCameraToObject(Pose3d objectInField, Transform3d robotToCamera, DoubleArrayEntry cameraToObjectEntry, DifferentialDrivetrainSim drivetrainSimulator){
    Pose3d robotInField = new Pose3d(drivetrainSimulator.getPose());
    Pose3d cameraInField = robotInField.plus(robotToCamera);
    Transform3d cameraToObject = new Transform3d(cameraInField, objectInField);

    double [] val = {
      cameraToObject.getX(),
      cameraToObject.getY(),
      cameraToObject.getZ(),
      cameraToObject.getRotation().getQuaternion().getW(),
      cameraToObject.getRotation().getQuaternion().getX(),
      cameraToObject.getRotation().getQuaternion().getY(),
      cameraToObject.getRotation().getQuaternion().getZ(),
    };

    cameraToObjectEntry.set(val);

  }

  public Pose3d objectToRobotPose(Pose3d objectInField, Transform3d robotToCamera, DoubleArrayEntry cameraToObjectEntry){
    double[] val = cameraToObjectEntry.get();
    Translation3d translation = new Translation3d(val[0], val[1], val[2]);
    Rotation3d rotation = new Rotation3d(new Quaternion(val[3], val[4], val[5], val[6]));
    Transform3d cameraToObject = new Transform3d(translation, rotation);

    return ComputerVisionUtil.objectToRobotPose(objectInField, cameraToObject, robotToCamera);
  }

  public void updateOdometry(){
    m_poseEstimator.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    publishCameraToObject(m_objectInField, m_robotToCamera, m_cameraToObjectEntry, m_drivetrainSimulator);
    Pose3d visionMeasurement3d = objectToRobotPose(m_objectInField, m_robotToCamera, m_cameraToObjectEntry);
    
  }

  public void driveArcade(double xForward, double zRotation){
    m_drive.arcadeDrive(xForward, zRotation);
  }

  public void driveRaw(double power){
    m_leftA.set(power);
    m_rightA.set(power);
  }

  public double getHeading(){//-180,180
    return Math.IEEEremainder(m_pigeon.getYaw(), 360);
  }


  public double getPosition(){
    return m_rightEncoder.getPosition();
  }

  public void turn(double power){
    m_leftA.set(power);
    m_rightA.set(-power);
  }

  public double modAngle(double angle){
    return Math.IEEEremainder(angle, 360);
  }

}
