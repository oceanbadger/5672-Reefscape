// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class Intake extends SubsystemBase {
    SparkFlex intakeMotor1, intakeMotor2;
    SparkMaxConfig intakeMotor1Config, intakeMotor2Config;

  /** Creates a new Intake. */

  public Intake() {
    intakeMotor1 = new SparkFlex(37, MotorType.kBrushless);
    intakeMotor2 = new SparkFlex(38, MotorType.kBrushless);

    intakeMotor1Config = new SparkMaxConfig();
    intakeMotor2Config = new SparkMaxConfig();


    intakeMotor1.configure(intakeMotor1Config.
      inverted(false).
      idleMode(IdleMode.kBrake), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);

    intakeMotor2.configure(intakeMotor2Config.
      idleMode(IdleMode.kBrake).
      follow(37), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters);


  }

  public Command moveIntake(Double velocity) {
    // Inline construction of command goes here.
    return run(
        () -> {
          
          intakeMotor1.set(velocity);
        });
  }

  public void stopIntake(boolean b) {
    intakeMotor1.set(0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    //this method will be called once per sheduler run during simulation
  }

}
