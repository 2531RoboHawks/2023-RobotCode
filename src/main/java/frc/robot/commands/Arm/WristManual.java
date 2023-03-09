// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Arm;
import frc.robot.subsystems.ArmSubsystem;

public class WristManual extends CommandBase {

  public final ArmSubsystem m_armSubsystem;
  public final Joystick wristController;
  public final PIDController elbowPIDController;
  public final int ShoulderAxis;
  public final int ElbowAxis;
  public final int WristAxis;


  /** Creates a new WistManual. */
  public WristManual(ArmSubsystem m_armSubsystem, Joystick wristController, int shoulderAxis, int elbowAxis, int wristAxis) {
    this.m_armSubsystem = m_armSubsystem;
    addRequirements(m_armSubsystem);

    this.wristController = wristController;
    this.ShoulderAxis = shoulderAxis;
    this.ElbowAxis = elbowAxis;
    this.WristAxis = wristAxis;

    this.elbowPIDController = new PIDController(Arm.elbowKP, Arm.elbowKI, Arm.elbowKD); //Input the PID values for the elbow
    elbowPIDController.setTolerance(2); //Sets the tolerance of the elbow PID Controller
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elbowPIDController.setSetpoint(m_armSubsystem.getElbowAngle()); //Sets the setpoint of the elbow PID Controller to the current position
    
    double eAxis = wristController.getRawAxis(ElbowAxis);
    double wAxis = wristController.getRawAxis(WristAxis);

    eAxis = (Math.abs(eAxis) < Constants.stickDeadband) ? 0 : eAxis;
    wAxis = (Math.abs(wAxis) < Constants.stickDeadband) ? 0 : wAxis;

    m_armSubsystem.setSpeeds(0, eAxis, 0.1*wAxis);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.stopMotor();
    System.out.println("Wrist motor stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
