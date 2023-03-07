// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.*;
import frc.robot.Constants.Swerve.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmMid extends CommandBase  {

  private final ArmSubsystem m_armSubsystem;
  private final PIDController shoulderPIDController;
  private final PIDController elbowPIDController;
  private final PIDController wristPIDController;

    /** Creates a new ArmMid. */
    public ArmMid(ArmSubsystem m_armSubsystem) {
          this.m_armSubsystem = m_armSubsystem;
          addRequirements(m_armSubsystem);

          this.shoulderPIDController = new PIDController(Arm.shoulderKP, Arm.shoulderKI, Arm.shoulderKD); //Input the PID values for the shoulder
          shoulderPIDController.setTolerance(2); //Sets the tolerance of the shoulder PID Controller
          shoulderPIDController.setSetpoint(Arm.shoulderMidPosition); //Sets the setpoint of the shoulder PID Controller to the Mid position

          this.elbowPIDController = new PIDController(Arm.elbowKP, Arm.elbowKI, Arm.elbowKD); //Input the PID values for the elbow
          elbowPIDController.setTolerance(2); //Sets the tolerance of the elbow PID Controller
          elbowPIDController.setSetpoint(Arm.elbowMidPosition); //Sets the setpoint of the elbow PID Controller to the Mid position

          this.wristPIDController = new PIDController(Arm.wristKP, Arm.wristKI, Arm.wristKD); //Input the PID values for the wrist
          wristPIDController.setTolerance(.5); //Sets the tolerance of the wrist PID Controller
          wristPIDController.setSetpoint(Arm.wristMidPosition); //Sets the setpoint of the wrist PID Controller to the Mid position
    }
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ArmMid command started. Everyone stand back, I know what I'm doing. Trust me.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulderSpeed = shoulderPIDController.calculate(m_armSubsystem.getShoulderAngle()); //Gets the angle of the shoulder and calculates the error
    double elbowSpeed = elbowPIDController.calculate(m_armSubsystem.getElbowAngle()); //Gets the angle of the elbow and calculates the error
    double wristSpeed = wristPIDController.calculate(m_armSubsystem.getWristAngle()); //Gets the angle of the wrist and calculates the error

    //Disable the shoulder motor cuz is always broken lel
    m_armSubsystem.setSpeeds(0, elbowSpeed, wristSpeed); //Sets the speeds of the motors based on the calculated error

    System.out.println("Shoulder Angle: " + m_armSubsystem.getShoulderAngle()); //Prints the angle of the shoulder to the console
    System.out.println("Elbow Angle: " + m_armSubsystem.getElbowAngle()); //Prints the angle of the elbow to the console
    System.out.println("Wrist Angle: " + m_armSubsystem.getWristAngle()); //Prints the angle of the wrist to the console

    SmartDashboard.putNumber("Shoulder Angle: ", m_armSubsystem.getShoulderAngle()); //Prints the angle of the shoulder to the SmartDashboard
    SmartDashboard.putNumber("Elbow Angle: ", m_armSubsystem.getElbowAngle()); //Prints the angle of the elbow to the SmartDashboard
    SmartDashboard.putNumber("Wrist Angle: ", m_armSubsystem.getWristAngle()); //Prints the angle of the wrist to the SmartDashboard
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.stopMotor(); //Stops the motors
    System.out.println("Arm Motors Stopped!!!!!"); //Prints to the console
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

