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
import edu.wpi.first.wpilibj.command.Command;

public class ArcadeDrive extends Command {
  private double deadZone = 0.1;
  private double turn =0;
  private double throttel = 0;
  private double ratio = 0;
  private double sensitivity = 0.75;
  private double leftPower;
  private double rightPower;
  public ArcadeDrive() {
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
	  ratio = Math.abs(throttel);
    throttel = OI.joyStickOne.getRawAxis(1);  
    if(Math.abs(OI.joyStickOne.getRawAxis(4))>deadZone) {	
    	turn = OI.joyStickOne.getRawAxis(4);
    }
    else {
    	turn = 0;
    }
    if(Math.abs(OI.joyStickOne.getRawAxis(1))>deadZone){
    	leftPower = (throttel - (sensitivity*turn*ratio));
    	rightPower = (throttel + (sensitivity*turn*ratio));
    }
    else{
    	leftPower = (-turn);
    	rightPower = (turn); 
    }
    if(OI.joyStickOne.getRawAxis(3)>0.5) {
    	leftPower = (-turn);
    	rightPower= (turn);
    }
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
    
   
    if(OI.shiftUp.get()) {
  		RobotMap.shifters.set(RobotMap.highGear);
  	}
  	else if(OI.shiftDown.get()) {
  		RobotMap.shifters.set(RobotMap.lowGear);
  	}
  	if(RobotMap.shifters.get() == RobotMap.highGear) {
  		for(TalonSRX talon:RobotMap.driveMotors) {
  	    	talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentHighGear, RobotConfig.timeOut);
  	    	talon.configPeakCurrentLimit(RobotConfig.driveMotorPeakCurrentHighGear, 0);  
  	    	talon.configPeakCurrentDuration(RobotConfig.driveMotorPeakCurrentDurationHighGear, 0);
  	    	talon.enableCurrentLimit(true);
  	    	}
        sensitivity =1.75;

  	}
  	else if(RobotMap.shifters.get() == RobotMap.lowGear) {
  		for(TalonSRX talon:RobotMap.driveMotors) {	
  			talon.configContinuousCurrentLimit(RobotConfig.driveMotorContinuousCurrentLowGear, RobotConfig.timeOut);
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}