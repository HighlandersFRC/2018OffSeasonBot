/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*---------------------------------------------------------------------------*/

package frc.robot.teleopcommands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.OI;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class ArcadeDrive extends Command {
  private double deadZone = 0.05;
  private double turn =0;
  private double throttel = 0;
  private double ratio = 0;
  private double sensitivity = 1.25;
  private double leftPower;
  private double rightPower;
  private double throttleJoystickValue;
  private double turnJoystickValue;
  private double triggerBrakeValue;
  public ArcadeDrive() {
    requires(RobotMap.drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    throttleJoystickValue = OI.controller1.getRawAxis(1);
    turnJoystickValue = OI.controller1.getRawAxis(4);
    triggerBrakeValue = OI.controller1.getRawAxis(3);
    throttel = throttleJoystickValue;  
    ratio = Math.abs(throttel);
    if(Math.abs(turnJoystickValue)>deadZone) {	
    	turn = turnJoystickValue;
    }
    else {
    	turn = 0;
    }
    if(Math.abs(throttleJoystickValue)>deadZone){
    	leftPower = (throttel - (sensitivity*turn*ratio));
    	rightPower = (throttel + (sensitivity*turn*ratio));
    }
    else{
    	leftPower = (-turn);
    	rightPower = (turn); 
    }
    if(triggerBrakeValue>0.5) {
    	leftPower = (-turn);
    	rightPower= (turn);
    }
    //Keeps left and right power between -1 and 1 and scales other value to match
    if(Math.abs(leftPower)>1) {
      leftPower = (leftPower/Math.abs(leftPower));
      rightPower = Math.abs(rightPower/leftPower)*(rightPower/Math.abs(rightPower));
    }
    else if(Math.abs(rightPower)>1) {
      rightPower = (rightPower/Math.abs(rightPower));
      leftPower = Math.abs(leftPower/rightPower)*(leftPower/Math.abs(leftPower));
    }
    
    RobotMap.leftDriveLead.set(ControlMode.PercentOutput, leftPower);
    RobotMap.rightDriveLead.set(ControlMode.PercentOutput, rightPower);
    
   
    if(OI.controller1.getBumper(Hand.kRight)) {
      RobotMap.drive.setHighGear();
    }
    else if(OI.controller1.getBumper(Hand.kLeft)) {
      RobotMap.drive.setLowGear();
    }
  	if(RobotMap.shifters.get() == RobotMap.highGear) {
  		for(TalonSRX talon:RobotMap.driveMotors) {
  	    	talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentHighGear, 0);
  	    	talon.configPeakCurrentLimit(RobotConfig.driveMotorPeakCurrentHighGear, 0);  
  	    	talon.configPeakCurrentDuration(RobotConfig.driveMotorPeakCurrentDurationHighGear, 0);
  	    	talon.enableCurrentLimit(true);
  	    	}
      sensitivity =1.75;
  	}
  	else if(RobotMap.shifters.get() == RobotMap.lowGear) {
  		for(TalonSRX talon:RobotMap.driveMotors) {	
  			  talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentLowGear, 0);
  	    	talon.configPeakCurrentLimit(RobotConfig.driveMotorPeakCurrentLowGear, 0);  
  	    	talon.configPeakCurrentDuration(RobotConfig.driveMotorPeakCurrentDurationLowGear, 0);
  	    	talon.enableCurrentLimit(true);
  	  }
      sensitivity =1.25;
  	}
   }
  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (!(RobotState.isOperatorControl()));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
      RobotMap.leftDriveLead.set(ControlMode.PercentOutput, 0);
      RobotMap.rightDriveLead.set(ControlMode.PercentOutput, 0);
  
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
