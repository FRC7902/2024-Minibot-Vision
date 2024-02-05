// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToDistanceSimple;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToTarget;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private static final PhotonCamera camera = new PhotonCamera("FirebirdsCamera");
  private static final CameraSubsystem m_cameraSubsystem = new CameraSubsystem(camera);
  private static final DriveSubsystem m_driveSubsystem = new DriveSubsystem(camera);
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    m_driveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_driveSubsystem.driveArcade(
          m_driverController.getRawAxis(1) * DriveConstants.speedMultiplier, 
          m_driverController.getRawAxis(0) * DriveConstants.speedMultiplier),
          m_driveSubsystem
      ));    
  }

  private void configureBindings() {
    new JoystickButton(m_driverController, IOConstants.kA).onTrue(new TurnToTarget(m_cameraSubsystem, m_driveSubsystem));
    // new JoystickButton(m_driverController, IOConstants.kB).onTrue(new TurnToAngle(m_driveSubsystem, 90, true));
    // new JoystickButton(m_driverController, IOConstants.kX).onTrue(new TurnToAngle(m_driveSubsystem, 0, false));
    // new JoystickButton(m_driverController, IOConstants.kY).onTrue(new driveShape2(m_driveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
