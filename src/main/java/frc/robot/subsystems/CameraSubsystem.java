// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class CameraSubsystem extends SubsystemBase {
  public PhotonPipelineResult result;
  public final PhotonCamera m_camera;
  private final double CameraHeightMetres = 0.44;
  private final double TargetHeightMetres = 1.35;
  private final double CameraPitchRadians = 0.17;
  private double tagDistance = 0;
  private double tagAngle = 0;
  private int ID;

  //simulation stuff

  double camDiagFOV = 68.5; // degrees - assume wide-angle camera
  double camPitch = 0.17; // degrees
  double camHeightOffGround = 0.44; // meters
  double maxLEDRange = 20; // meters
  int camResolutionWidth = 1280; // pixels
  int camResolutionHeight = 720; // pixels
  double minTargetArea = 10; // square pixels
//test
  public CameraSubsystem(PhotonCamera camera) {
    m_camera = camera;
  }

  @Override
  public void periodic() {
    result = m_camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    List<PhotonTrackedTarget> targets = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();

    if(hasTargets){
      tagAngle = target.getYaw();
      ID = target.getFiducialId();
      tagDistance = PhotonUtils.calculateDistanceToTargetMeters(CameraHeightMetres, TargetHeightMetres, CameraPitchRadians, Units.degreesToRadians(result.getBestTarget().getPitch()));
    }


    SmartDashboard.putNumber("AprilTag target angle", tagAngle);
    SmartDashboard.putBoolean("Has targets", result.hasTargets());
    SmartDashboard.putNumber("Apriltag ID", ID);
    SmartDashboard.putNumber("Apriltag distance",tagDistance);

  }

  public double returnTargetYaw(){
    return tagAngle;
  }

}
