// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ManageLimeLightCMD;
//import frc.robot.commands.MoveArmCMD;
import frc.robot.commands.SwerveJoystickCmd;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSub;
import frc.robot.subsystems.LimelightSub;

import com.fasterxml.jackson.core.io.IOContext;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;




public class RobotContainer {

  // private final CommandXboxController m_mechController =
  //     new CommandXboxController(OIConstants.kDriverControllerPort);


  private final SwerveSub swerveSub =  new SwerveSub();
 // private final ArmSub armsub = new ArmSub();
  private final LimelightSub limelightSub = new LimelightSub();
  private final Joystick driverJoyStick = new Joystick(OIConstants.kDriverControllerPort);
  private final CommandPS4Controller m_mechController = 
    new CommandPS4Controller(1);
  private final Intake m_intake = new Intake();
  private final Arm m_arm = new Arm();






  public RobotContainer() {
    
    

    // Configure the trigger bindings
    swerveSub.setDefaultCommand(
        new SwerveJoystickCmd(
        swerveSub,
        () -> -driverJoyStick.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverJoyStick.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverJoyStick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverJoyStick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
        () -> driverJoyStick.getRawButton(OIConstants.kOrientToTargetIdx)
        )
      ); // by defualt will work on fields reference frame
      
    limelightSub.setDefaultCommand(
      new ManageLimeLightCMD(limelightSub)
    );

    configureBindings();
  }


  private void configureBindings() {
    //new JoystickButton(driverJoyStick, OIConstants.kMoveArmIdx ).whileTrue(new MoveArmCMD(armsub));

    //**** Intake Bindings  ******

    // Default behaviour (do nothing)
    m_intake.setDefaultCommand(m_intake.moveIntake(0.0));

    // Run intake with right bumper button 5
    m_mechController.button(5)
      .and(m_mechController.button(6).negate())
      .whileTrue(m_intake.moveIntake(0.75));

    // Run intake in reverse with left bumper button 6
    m_mechController.button(6)
      .and(m_mechController.button(5).negate()) 
      .whileTrue(m_intake.moveIntake(-0.75));

       // *** Arm bindings ***

    // Move to intake coral with Y
    m_mechController.button(4)
    .onTrue(m_arm.moveArmToPosition(ArmConstants.positionIntakeCoral));

  // Move to intake algae with X  button 3 SWITCHED TO button 5 to test
    m_mechController.button(3)
    .onTrue(m_arm.moveArmToPosition(ArmConstants.positionIntakeAlgae));

  // Move to remove low-reef algae and dump L1 coral with B
    m_mechController.button(2)
    .onTrue(m_arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeLow));

  // Move to remove high-reef algae with A
    m_mechController.button(1)
    .onTrue(m_arm.moveArmToPosition(ArmConstants.positionRemoveAlgaeHigh));

  // Move to start climb with D-Pad Down
    m_mechController.povDown()
    .onTrue(m_arm.moveArmToPosition(ArmConstants.positionClimbStart));

  // Move to finish climb with D-Pad up
    m_mechController.povUp()
    .onTrue(m_arm.moveArmToPosition(ArmConstants.positionClimbEnd));



    
  }


  public Command getAutonomousCommand() {


    return new PathPlannerAuto("Auto_driveForwardAndMoveArm");


  }
}