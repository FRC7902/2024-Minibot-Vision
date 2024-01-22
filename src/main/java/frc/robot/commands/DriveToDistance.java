// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private final double targetDistance;

  private final PIDController drivePID = new PIDController(1, 0, 0);
  private double initialPositionX;
  private double initialPositionY;
  private double angle;
  private double multiplier;
  private int direction;

  /** Creates a new DriveToDistance. */
  public DriveToDistance(DriveSubsystem drive, double distance) {
    m_driveSubsystem = drive;
    targetDistance = distance;
    drivePID.setTolerance(0.01, 1);
    //addRequirements(drive);
    
    initialPositionX = m_driveSubsystem.getDisplacementX();
    initialPositionY = m_driveSubsystem.getDisplacementY();
    angle = m_driveSubsystem.getHeading();

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    initialPositionX = m_driveSubsystem.getDisplacementX();
    initialPositionY = m_driveSubsystem.getDisplacementY();
    angle = m_driveSubsystem.getHeading();

    if((45 < angle && angle < 135) || (-135 < angle && angle < -45)){
      multiplier = sin(angle);
    }else{
      multiplier = cos(angle);
    }

    if(multiplier < 0){
      direction = -1;
    }else{
      direction = 1;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;


    if(multiplier == sin(angle)){
      speed = drivePID.calculate(m_driveSubsystem.getDisplacementY(), initialPositionY + (targetDistance * multiplier));
    }else{
      speed = drivePID.calculate(m_driveSubsystem.getDisplacementX(), initialPositionX + (targetDistance * multiplier));
    }

    m_driveSubsystem.driveRaw(direction * speed);

    SmartDashboard.putNumber("Current Pos", m_driveSubsystem.getDisplacementX());
    SmartDashboard.putNumber("Target Pos", targetDistance);

  }

  public double cos(double angle){
    return Math.cos(Math.toRadians(angle));
  }

  public double sin(double angle){
    return Math.sin(Math.toRadians(angle));
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