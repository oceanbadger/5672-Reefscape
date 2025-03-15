package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Arm;

//public class MoveArmCMD extends Command{
    //public final Arm arm; 
    //public final SparkFlex armMotor;
    //public final PIDController armController;


    
    //public MoveArmCMD(Arm arm){
        //this.arm = arm;
        //this.armMotor = arm.getMotor();
        //armController = arm.getArmController();
        //addRequirements(arm);
        
    //}

    //@Override
    //public void initialize(){
        

        //armMotor.set(0);
        //armMotor.stopMotor();
        
        //SmartDashboard.putBoolean("isArmCommandRunning", true);
    //}

    
    //@Override
    //public void execute(){
        //telemetry
        //SmartDashboard.putData(armController);
        //SmartDashboard.putNumber("armPostionError_degrees",armController.getError());
        //SmartDashboard.putNumber("armPostion_degrees",arm.getGetArmEncoderPosition_degrees());
        //drive arm Motor to setpoint based on arm controller
        //double output = armController.calculate(arm.getGetArmEncoderPosition_degrees(), 169);
        //armMotor.set(output);       
    //}
    //@Override
    //public void end(boolean interrupted){
        //SmartDashboard.putBoolean("isArmCommandRunning", false);
        //armMotor.set(0);
        //armMotor.stopMotor();
    //}
    //@Override
   // public boolean isFinished(){
        //
       // if(Math.abs(armController.getError()) <= 3){
            //return true;
        //}
        //return false;
    //}
//}