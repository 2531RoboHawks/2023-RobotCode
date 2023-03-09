// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BDSM license file in the root directory of this project.  

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory; 

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Claw.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Creates new joystick object for the driver on port 0
  private final Joystick driver = new Joystick(0);
  private final Joystick arm = new Joystick(1);

  // Creates the Axis variables mapped to various joysticks on the controller
  private final int translationAxis = XboxController.Axis.kLeftY.value; //Y axis on left joystick, front to back motion
  private final int strafeAxis = XboxController.Axis.kLeftX.value; //X axis on the left joystick, left to right motion
  private final int rotationAxis = XboxController.Axis.kRightX.value; //X axis on the right joystick, turns the robot
  private final int wristAxis = XboxController.Axis.kLeftY.value;
  private final int gripperAxis = 4;

  // Creates button mappings on the controller
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); // Y button on the controller to zero the gyro
  private final int slowMode = XboxController.Button.kB.value;
  private final JoystickButton autoBalance = new JoystickButton(driver, XboxController.Button.kA.value);
  //Xbox controller values
  //private final JoystickButton armUpAndOut = new JoystickButton(arm, XboxController.Button.kY.value); // Arm up and out
  private final JoystickButton armDownAndOut = new JoystickButton(arm, XboxController.Button.kA.value); // Arm down and out
  private final JoystickButton armStore = new JoystickButton(arm, XboxController.Button.kB.value); // Default position
  private final JoystickButton armMiddle = new JoystickButton(arm, XboxController.Button.kX.value); // Place object in middle row
  private final JoystickButton motorRelease = new JoystickButton(arm, 7); //Arm free fall
  private final JoystickButton zeroArmEncoders = new JoystickButton(arm, 8); 
  private final JoystickButton gripperOpen = new JoystickButton(arm, XboxController.Button.kLeftBumper.value); //Opens claw
  private final JoystickButton gripperClose = new JoystickButton(arm, XboxController.Button.kRightBumper.value); //Close claw

  private final Swerve swerveSubsystem = new Swerve();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ArmHigh armHigh = new ArmHigh(armSubsystem);
  private final ArmLow armLow = new ArmLow(armSubsystem);
  private final PutThoseGrippersAway armStow = new PutThoseGrippersAway(armSubsystem);
  private final ArmMid armMid = new ArmMid(armSubsystem);
  private final ChargeBalance chargeBalance = new ChargeBalance(swerveSubsystem);

  private final GripperOpen greasyGripper9000Open = new GripperOpen(armSubsystem);
  private final GripperClose greasyGripper9000Close = new GripperClose(armSubsystem);


      /* Autonomous Mode Chooser */
      private final SendableChooser<Command> autoChooser = new SendableChooser<>();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true; // Do you want field oriented control?
    boolean openLoop = true; 
    swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop, slowMode));  //Default command to drive the bot
    // Configure the button bindings
    configureButtonBindings();

    configureSmartDashboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Drive Buttons
    zeroGyro.onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    autoBalance.whileTrue(chargeBalance);
    
    //Arm Buttons
    armUpAndOut.onTrue(armHigh);
    armDownAndOut.onTrue(armLow);
    armStore.onTrue(armStow);
    armMiddle.onTrue(armMid);

    //Gripper Buttons
    gripperOpen.onTrue(greasyGripper9000Open);
    gripperClose.onTrue(greasyGripper9000Close);

    //Debug Buttons
    motorRelease.onTrue(new InstantCommand(() -> armSubsystem.releaseAllMotors()));
    motorRelease.onFalse(new InstantCommand(() -> armSubsystem.brakeAllMotors()));
    zeroArmEncoders.onTrue(new InstantCommand(() -> armSubsystem.zeroAllEncoders()));
  }

  
  private void configureSmartDashboard() {
    //Add auton routine options to the SmartDashboard SendableChooser
    autoChooser.setDefaultOption("Simple Straight Out", new SequentialCommandGroup(
      new SimpleOut(swerveSubsystem)));

    autoChooser.addOption("Straight out to Dock", new SequentialCommandGroup(
      new StraightDock(swerveSubsystem)));

    autoChooser.addOption("Place Cube in Mid then Leave", new SequentialCommandGroup(
      new DropCubeMid(swerveSubsystem, armSubsystem)));

    autoChooser.addOption("Straight out to Dock with Balance", new SequentialCommandGroup(
      new StraightDockWithAutoBal(swerveSubsystem)));

    autoChooser.addOption("Do absolutly nothing", null);
    
    SmartDashboard.putData(autoChooser);
  }

  public void disabledInit() {
    swerveSubsystem.resetModulesToAbsolute();
  }

  public void robotInit() {
    armSubsystem.zeroAllEncoders();
    armSubsystem.init();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //Run the selected routine from the SmartDashboard SendableChooser
        return autoChooser.getSelected();
  }
}
