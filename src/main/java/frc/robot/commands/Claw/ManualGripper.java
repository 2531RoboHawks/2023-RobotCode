// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Arm;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.Joystick;

public class ManualGripper extends CommandBase {
  /** Creates a new ManualGripper. */
  private final ArmSubsystem m_armSubsystem;
  private final Joystick gripperController;
  //private final PIDController shoulderPIDController;
  private final PIDController elbowPIDController;
  private final PIDController wristPIDController;
  //private final int ShoulderAxis;
  private final int ElbowAxis;
  private final int WristAxis;

  private final int gripperAxis;

  public ManualGripper(ArmSubsystem m_armSubsystem, Joystick gripperController, int gripperAxis, int elbowAxis, int wristAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_armSubsystem = m_armSubsystem;
    addRequirements(m_armSubsystem);

    this.gripperController = gripperController;
    this.gripperAxis = gripperAxis;
    //this.ShoulderAxis = shoulderAxis;
    this.ElbowAxis = elbowAxis;
    this.WristAxis = wristAxis;

    
    //this.shoulderPIDController = new PIDController(Arm.shoulderKP, Arm.shoulderKI, Arm.shoulderKD); //Input the PID values for the shoulder
    //shoulderPIDController.setTolerance(2); //Sets the tolerance for the sholder PID controller

    this.elbowPIDController = new PIDController(Arm.elbowKP, Arm.elbowKI, Arm.elbowKD); //Input the PID values for the elbow
    elbowPIDController.setTolerance(2); //Sets the tolerance of the elbow PID Controller

    this.wristPIDController = new PIDController(Arm.wristKP, Arm.wristKI, Arm.wristKD); //Input the PID values for the wrist
    wristPIDController.setTolerance(2); //Sets the tolerance of the wrist PID Controller
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shoulderPIDController.setSetpoint(m_armSubsystem.getShoulderAngle()); //Sets the setpoint for the shoulder PID controller
    elbowPIDController.setSetpoint(m_armSubsystem.getElbowAngle()); //Sets the setpoint of the elbow PID Controller to the current position
    wristPIDController.setSetpoint(m_armSubsystem.getWristAngle()); //Sets the setpoint of the wrist PID Controller to the current position
  }
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gripperPosition = m_armSubsystem.getGripperPosition();
    double gripperSpeed = gripperController.getRawAxis(gripperAxis);

    if(gripperPosition <= Arm.gripperOpenPosition+10000 && gripperSpeed < 0) {
      gripperSpeed = 0;
      m_armSubsystem.stopGripper();
      //System.out.println("Gripper opened all the way!!!!");
    }
    //double shoulderSpeed = shoulderPIDController.calculate(m_armSubsystem.getShoulderAngle()); //Gets the angle of the elbow and calculates the error
    double elbowSpeed = elbowPIDController.calculate(m_armSubsystem.getElbowAngle()); //Gets the angle of the elbow and calculates the error
    double wristSpeed = wristPIDController.calculate(m_armSubsystem.getWristAngle()); //Gets the angle of the wrist and calculates the error

    gripperSpeed = 1 * ((Math.abs(gripperSpeed) < Constants.stickDeadband) ? 0 : gripperSpeed);

    m_armSubsystem.setGripperSpeed(gripperSpeed);
    m_armSubsystem.setSpeeds(0, elbowSpeed, wristSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Gripper Manual Command Stopped!");
    m_armSubsystem.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
