// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleSimple extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetDistance;
  private final PIDController turnPID = new PIDController(1, 0.25, 0);
  private double initialPosition;

  /** Creates a new TurnToAngleSimple. */
  public TurnToAngleSimple(DriveSubsystem drive, double targetDegrees) {
    m_driveSubsystem = drive;
    targetDistance = (targetDegrees/360)*DriveConstants.DistanceBetweenWheels*Math.PI;
    turnPID.setTolerance(0.01, 1);
    initialPosition = m_driveSubsystem.getPosition();
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

    speed = turnPID.calculate(m_driveSubsystem.getPosition(), initialPosition + targetDistance);
    m_driveSubsystem.turn(-speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnPID.atSetpoint();
  }
}
