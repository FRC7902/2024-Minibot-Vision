// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  public PhotonPipelineResult result;
  public final PhotonCamera m_camera;
  private double targetAngle = 0;
  private int ID;
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
      targetAngle = target.getYaw();
      ID = target.getFiducialId();

    }


    SmartDashboard.putNumber("AprilTag target angle", targetAngle);
    SmartDashboard.putBoolean("Has targets", result.hasTargets());
    SmartDashboard.putNumber("Apriltag ID", ID);
  }

  public double returnTargetYaw(){
    return targetAngle;
  }

}
