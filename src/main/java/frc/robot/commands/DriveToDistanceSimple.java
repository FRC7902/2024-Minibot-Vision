// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistanceSimple extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetDistance;
  private final PIDController drivePID = new PIDController(1, 0, 0);
  private double initialPosition;

  /** Creates a new DriveToDistanceSimple. */
  public DriveToDistanceSimple(DriveSubsystem drive, double distance) {

    m_driveSubsystem = drive;
    targetDistance = distance;
    drivePID.setTolerance(0.01, 1);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPosition = m_driveSubsystem.getPosition();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;

    speed = drivePID.calculate(m_driveSubsystem.getPosition(), initialPosition + targetDistance);

    m_driveSubsystem.driveRaw(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePID.atSetpoint();
  }
}
