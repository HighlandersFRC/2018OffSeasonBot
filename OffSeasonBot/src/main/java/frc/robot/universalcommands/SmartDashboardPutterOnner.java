/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.universalcommands;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardPutterOnner extends Command {
  //Question, have you noticed how curly the k is vs code is like I love this also why does the l look so weird?
  public SmartDashboardPutterOnner() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }
  private void teleopSmartDashboard(){
    SmartDashboard.putBoolean("navxconnection",RobotMap.mainNavx.isOn());

		if(RobotMap.mainNavx.isOn()){
			SmartDashboard.putBoolean("navxCallibration",RobotMap.mainNavx.isCalibrated());

		}
		SmartDashboard.putNumber("navx", RobotMap.mainNavx.currentAngle());
    
    // all of the things that need to be updated in teleop
  }
  private void autoSmartDashboard(){
    // all of the things that need to be updated in auto
  }
  private void disabledSmartDashboard(){
    SmartDashboard.putNumber("distancer", RobotMap.rightMainDrive.getDistance());
    SmartDashboard.putNumber("distancel", RobotMap.leftMainDrive.getDistance());
    SmartDashboard.putBoolean("navxconnection",RobotMap.mainNavx.isOn());
    // all of the things that need to be updated when the robot is disabled
  }
  private void enabledSmartDashboard(){
    SmartDashboard.putBoolean("navxconnection",RobotMap.mainNavx.isOn());
    // all of the things that need to be updated when the robot is just enabled and boring

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(RobotState.isEnabled()){
      enabledSmartDashboard();
    }
    if(RobotState.isOperatorControl()){
      teleopSmartDashboard();
    }
    else if(RobotState.isAutonomous()){
      autoSmartDashboard();
    }
    else if(RobotState.isDisabled()){
      disabledSmartDashboard();
    }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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