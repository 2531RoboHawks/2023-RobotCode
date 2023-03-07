// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.Swerve.Arm;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristManual extends CommandBase {

  //private final ArmSubsystem armSubsystem;
  private final Joystick wristController;
  private final int ShoulderAxis;
  private final int WristAxis;
  private final int ElbowAxis;
  private final PIDController shoulderPIDController;
  private final PIDController elbowPIDController;
  private final ArmSubsystem m_armSubsystem;
  private final boolean ArmPosition;

  /** Creates a new WristManual. */
  public WristManual(ArmSubsystem m_armSubsystem, Joystick wristController, int shoulderAxis, int elbowAxis, int wristAxis) {
    this.m_armSubsystem = m_armSubsystem;
    addRequirements(m_armSubsystem);

    this.wristController = wristController;
    this.ShoulderAxis = shoulderAxis;
    this.ElbowAxis = elbowAxis;
    this.WristAxis = wristAxis;
    //this.ArmPosition = armPosition;
    
    //this.shoulderPIDController = new PIDController(Arm.shoulderKP, Arm.shoulderKI, Arm.shoulderKD); //Input the PID values for the shoulder
    //shoulderPIDController.setTolerance(2); //Sets the tolerance of the shoulder PID Controller

    this.elbowPIDController = new PIDController(Arm.elbowKP, Arm.elbowKI, Arm.elbowKD); //Input the PID values for the elbow
    elbowPIDController.setTolerance(2); //Sets the tolerance of the elbow PID Controller
    elbowPIDController.setSetpoint(Arm.elbowMidPosition); //Sets the setpoint of the elbow PID Controller to the Mid position

    /*if(armPosition = true){
      elbowPIDController.setSetpoint(Arm.elbowHighPosition); //Sets the setpoint of the elbow PID Controller to the High position
      shoulderPIDController.setSetpoint(Arm.shoulderHighPosition); //Sets the setpoint of the shoulder PID Controller to the High position

    }if(armPosition = true) {
      elbowPIDController.setSetpoint(Arm.elbowMidPosition); //Sets the setpoint of the elbow PID Controller to the Mid position
      shoulderPIDController.setSetpoint(Arm.shoulderMidPosition); //Sets the setpoint of the shoulder PID Controller to the Mid position

    }if(armPosition = true){
      elbowPIDController.setSetpoint(Arm.elbowLowPosition); //Sets the setpoint of the elbow PID Controller to the Low position
      shoulderPIDController.setSetpoint(Arm.shoulderLowPosition); //Sets the setpoint of the shoulder PID Controller to the Low position

    }
    */
    


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    double shoulderSpeed = shoulderPIDController.calculate(m_armSubsystem.getShoulderAngle()); //Gets the angle of the shoulder and calculates the error
    double elbowSpeed = elbowPIDController.calculate(m_armSubsystem.getElbowAngle()); //Gets the angle of the elbow and calculates the error
    
    double wAxis = wristController.getRawAxis(WristAxis);
    

    wAxis = (Math.abs(wAxis) < Constants.stickDeadband) ? 0 : wAxis;
    //eAxis = (Math.abs(wAxis) < Constants.stickDeadband) ? 0 : eAxis;

<<<<<<< HEAD
    m_armSubsystem.setSpeeds(shoulderSpeed, elbowSpeed, 0.1*wAxis);

    SmartDashboard.putNumber("Wrist Angle: ", m_armSubsystem.getShoulderAngle()); //Puts the shoulder angle on the SmartDashboard

=======
    armSubsystem.setSpeeds(0,0, 0.1*wAxis);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> parent of 83f76de (Merge branch 'AH-Duluth' of https://github.com/2531RoboHawks/2023-RobotCode into AH-Duluth)
=======
>>>>>>> parent of 83f76de (Merge branch 'AH-Duluth' of https://github.com/2531RoboHawks/2023-RobotCode into AH-Duluth)
=======
>>>>>>> parent of 83f76de (Merge branch 'AH-Duluth' of https://github.com/2531RoboHawks/2023-RobotCode into AH-Duluth)
=======
>>>>>>> parent of 83f76de (Merge branch 'AH-Duluth' of https://github.com/2531RoboHawks/2023-RobotCode into AH-Duluth)
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    m_armSubsystem.stopMotor(); //Stops the arm motors
    System.out.println("Arm Motors Stopped!!!!!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
