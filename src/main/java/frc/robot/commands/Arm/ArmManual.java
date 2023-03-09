// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class ArmManual extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final Joystick controller;
  private final int shoulderAxis;
  private final int elbowAxis;
  private final int wristAxis;
  

  /** Creates a new ArmHigh. */
  public ArmManual(ArmSubsystem armSubsystem, Joystick controller, int shoulderAxis, int elbowAxis, int wristAxis) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    //Defining variables
    this.controller = controller;
    this.shoulderAxis = shoulderAxis;
    this.elbowAxis = elbowAxis;
    this.wristAxis = wristAxis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ArmManual command started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double sAxis = controller.getRawAxis(shoulderAxis); //getting axis on controller for shoulder.
    double eAxis = controller.getRawAxis(elbowAxis); //getting axis on controller for elbow.
    double wAxis = controller.getRawAxis(wristAxis); //getting axis on controller for wrist.

    //Applying stick deadband
    sAxis = (Math.abs(sAxis) < Constants.stickDeadband) ? 0 : sAxis; 
    eAxis = (Math.abs(eAxis) < Constants.stickDeadband) ? 0 : eAxis;
    wAxis = (Math.abs(wAxis) < Constants.stickDeadband) ? 0 : wAxis;

    armSubsystem.setSpeeds(sAxis, eAxis, wAxis);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopMotor();
    System.out.println("Arm Motors Stopped!!!!!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
