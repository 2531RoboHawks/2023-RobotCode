// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.Swerve.Arm;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class WristManual extends CommandBase {
  
  private final ArmSubsystem m_armSubsystem;
  private final Joystick controller;
  private final int wristAxis;

  private final PIDController shoulderPIDController;
  private final PIDController elbowPIDController;

  /** Creates a new WristManual. */
  public WristManual(ArmSubsystem m_armSubsystem, Joystick controller, int wristAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_armSubsystem = m_armSubsystem;
    addRequirements(m_armSubsystem);

    this.controller = controller;
    this.wristAxis = wristAxis;

    this.shoulderPIDController = new PIDController(Arm.shoulderKP, Arm.shoulderKI, Arm.shoulderKD); //Input the PID values for the shoulder
    shoulderPIDController.setTolerance(2); //Sets the tolerance of the shoulder PID Controller
    shoulderPIDController.setSetpoint(m_armSubsystem.getShoulderAngle()); //Sets the setpoint of the shoulder PID Controller to the Mid position

    this.elbowPIDController = new PIDController(Arm.elbowKP, Arm.elbowKI, Arm.elbowKD); //Input the PID values for the elbow
    elbowPIDController.setTolerance(2); //Sets the tolerance of the elbow PID Controller
    elbowPIDController.setSetpoint(m_armSubsystem.getElbowAngle()); //Sets the setpoint of the elbow PID Controller to the Mid position

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("WristManual command activated");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulderSpeed = shoulderPIDController.calculate(m_armSubsystem.getShoulderAngle()); //Gets the angle of the shoulder and calculates the error
    double elbowSpeed = elbowPIDController.calculate(m_armSubsystem.getElbowAngle()); //Gets the angle of the elbow and calculates the error

    double wAxis = controller.getRawAxis(wristAxis);
    
    wAxis = (Math.abs(wAxis) < Constants.stickDeadband) ? 0 : wAxis;
    m_armSubsystem.setSpeeds(shoulderSpeed, elbowSpeed, 0.1*wAxis);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
