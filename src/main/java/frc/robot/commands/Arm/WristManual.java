// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristManual extends CommandBase {

  private final ArmSubsystem armSubsystem;
  private final Joystick controller;
  private final int WristAxis;

  /** Creates a new WristManual. */
  public WristManual(ArmSubsystem armSubsystem, Joystick controller, int wristAxis) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);

    this.controller = controller;
    this.WristAxis = wristAxis;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double wAxis = controller.getRawAxis(WristAxis);

    wAxis = (Math.abs(wAxis) < Constants.stickDeadband) ? 0 : wAxis;

    armSubsystem.setSpeeds(0,0, 0.1*wAxis);

    SmartDashboard.putNumber("Wrist Angle: ", armSubsystem.getShoulderAngle()); //Puts the shoulder angle on the SmartDashboard

  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    armSubsystem.stopMotor(); //Stops the arm motors
    System.out.println("Arm Motors Stopped!!!!!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
