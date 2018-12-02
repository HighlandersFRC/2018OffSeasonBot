/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import java.util.Vector;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.sensors.DriveEncoder;
import frc.robot.sensors.Navx;
import jaci.pathfinder.Pathfinder;

public class Odometry extends Command {
  private double theta;
  private double thetaNext;
  private Navx navx;
  private double leftSideNext;
  private double leftSide;
  private double leftDelta;
  private DriveEncoder leftDriveEncoder;
  private double rightSideNext;
  private double rightSide;
  private double rightDelta;
  private DriveEncoder rightDriveEncoder;
  private double centerVelocity;
  private double x;
  private double y;
  private double yNext;
  private double xNext;
  private Notifier odometryrunner;
  private double dt;
  public Odometry() {
    leftDriveEncoder = new DriveEncoder(RobotMap.leftDriveLead, RobotMap.leftDriveLead.getSelectedSensorPosition(0));
    rightDriveEncoder = new DriveEncoder(RobotMap.rightDriveLead, RobotMap.rightDriveLead.getSelectedSensorPosition(0));
    navx = new Navx(RobotMap.navx);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
  private class OdometryRunnable implements Runnable{
    //this is a sperate section of code that runs at a different update rate than the rest, this is necessary to match the dt
     public void run(){
      leftSideNext = leftDriveEncoder.getDistance();
      rightSideNext = rightDriveEncoder.getDistance();
      thetaNext = navx.currentAngle();
      leftDelta = (leftSideNext-leftSide);
      rightDelta = (rightSideNext-rightSide);
      centerVelocity = (leftDelta+rightDelta)/2;
      xNext = x+centerVelocity*Math.cos(Pathfinder.d2r(theta));
      yNext = y+centerVelocity*Math.sin(Pathfinder.d2r(theta));
      x = xNext;
      y = yNext;
      theta = thetaNext;
      leftSide = leftSideNext;
      rightSide = rightSideNext;
     }
 
   }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    odometryrunner = new Notifier(new OdometryRunnable());
    dt = 0.01;
    odometryrunner.startPeriodic(dt);
    navx.softResetAngle(navx.currentAngle());
    navx.softResetYaw(navx.currentYaw());
    leftDriveEncoder.softReset();
    rightDriveEncoder.softReset();
  }

  public void zero(){
    x =0;
    y = 0;
    theta = 0;
    navx.softResetYaw(navx.currentYaw());
    leftDriveEncoder.softReset();
    rightDriveEncoder.softReset();
  }
  public double getX(){
    return x;
  }
  public double gety(){
    return y;
  }
  public double gettheta(){
    return theta;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(!RobotState.isAutonomous()){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    odometryrunner.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  
  }
}
