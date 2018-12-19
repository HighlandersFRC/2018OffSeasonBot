/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;



import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.tools.Point;
import frc.robot.tools.Vector;
import frc.robot.RobotConfig;
import jaci.pathfinder.Pathfinder;

public class PurePursuitController extends Command {
  private PathSetup chosenPath;
  private Odometry odometry;
  private int closestSegment;
  private Point lookAheadPoint;
  private Point lastLookAheadPoint;
  private Notifier notifier;
  private int startingNumber;
  private double deltaX;
  private double deltaY;
  private double distToPoint;
  private double minDistanceToPoint;
  private Point closestPoint;
  private double lookAheadDistance;
  private DriveTrainVelocityPID leftDriveTrainVelocityPID;
  private DriveTrainVelocityPID rightDriveTrainVelocityPID;
  private double desiredRobotCurvature;
  private double kValue;
  private double curveAdjustedVelocity;
  private int lookAheadPointInt;
  
  public PurePursuitController(PathSetup path) {
    chosenPath = path;
    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    odometry = new Odometry(chosenPath.getReversed());
    lookAheadPoint = new Point(0, 0);
    closestPoint = new Point(0,0);
    lastLookAheadPoint = new Point(0,0);
    closestSegment = 0;
    minDistanceToPoint = 100;
    lookAheadDistance = 1.5;
    startingNumber = 0;
    kValue = 3;
    lookAheadPointInt = 0;
    leftDriveTrainVelocityPID = new DriveTrainVelocityPID(0, RobotMap.leftDriveLead, 1, 0.0402026, 0.18, 0.0006, 0.80);
    rightDriveTrainVelocityPID = new DriveTrainVelocityPID(0, RobotMap.rightDriveLead, 1, 0.0406258, 0.18, 0.0006, 0.80);
    leftDriveTrainVelocityPID.start();
    rightDriveTrainVelocityPID.start();
    odometry.start();
    odometry.zero();
    notifier = new Notifier(new PathRunnable());
    notifier.startPeriodic(0.005);
  }
  private class PathRunnable implements Runnable{
    public void run(){
      for(int i = startingNumber; i<chosenPath.getMainPath().length();i++){        
        deltaX = chosenPath.getMainPath().get(i).x-odometry.getX();
        deltaY = chosenPath.getMainPath().get(i).y-odometry.getY();
        distToPoint = Math.sqrt(Math.pow(deltaX, 2)+Math.pow(deltaY, 2));
        if(distToPoint<minDistanceToPoint){
          minDistanceToPoint = distToPoint;
          closestSegment = i;
          closestPoint.setLocation(chosenPath.getMainPath().get(i).x, chosenPath.getMainPath().get(i).y);
        }
      }
      SmartDashboard.putNumber("closestSegment", closestSegment);
      startingNumber = closestSegment;
      minDistanceToPoint = 100;
      for(int i = chosenPath.getMainPath().length()-1; i>=closestSegment;i--){
        Vector dist = new Vector();
        dist.setX(chosenPath.getMainPath().get(i).x - odometry.getX());
        dist.setY(chosenPath.getMainPath().get(i).y - odometry.getY());
        SmartDashboard.putNumber("dist", dist.length());
        if(dist.length()<lookAheadDistance){
          lookAheadPointInt = i;
          lookAheadPoint.setLocation(chosenPath.getMainPath().get(i).x, chosenPath.getMainPath().get(i).y);
        }
        if(i == closestSegment){
          lookAheadPoint = lastLookAheadPoint;
        }
      }
      lastLookAheadPoint = lookAheadPoint;
      SmartDashboard.putNumber("lookaheadX", lookAheadPoint.getXPos());
      SmartDashboard.putNumber("lookaheadY", lookAheadPoint.getYPos());
      SmartDashboard.putNumber("odometryx", odometry.getX());
      SmartDashboard.putNumber("odometryy", odometry.getY());
      
      findRobotCurvature();
     
      setWheelVelocities(chosenPath.getMainPath().get(closestSegment).velocity, desiredRobotCurvature);
    }
  } 
  private void setWheelVelocities(double targetVelocity, double curvature){
    double leftVelocity;
    double rightVelocity;
    double v = targetVelocity;
    if(v<1){
      v=-1;
    }
    double c = curvature;
    leftVelocity = v*(2+(c*RobotConfig.robotBaseDist))/2;
    rightVelocity = v*(2-(c*RobotConfig.robotBaseDist))/2;
    SmartDashboard.putNumber("leftVelocity", leftVelocity);
    SmartDashboard.putNumber("rightVelcotiy", rightVelocity);
    SmartDashboard.putNumber("velocity",targetVelocity);
    SmartDashboard.putNumber("curvature", curvature);
  
    leftDriveTrainVelocityPID.changeDesiredSpeed(leftVelocity);
    rightDriveTrainVelocityPID.changeDesiredSpeed(rightVelocity);
    
  }
  private void findRobotCurvature(){
    double a = -Math.tan(Pathfinder.d2r(odometry.gettheta()));
    double b = 1;
    double c = Math.tan(Pathfinder.d2r(odometry.gettheta())) * odometry.getX() - odometry.getY();
    double x = Math.abs( a * lookAheadPoint.getXPos()+ b * lookAheadPoint.getYPos() + c) /Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2));
    double side = Math.signum(Math.sin(Pathfinder.d2r(odometry.gettheta())) * (lookAheadPoint.getXPos()-odometry.getX())-Math.cos(Pathfinder.d2r(odometry.gettheta()))*(lookAheadPoint.getYPos()-odometry.getY())); 
    double curvature = ((2*x)/Math.pow(lookAheadDistance,2))*side;
    desiredRobotCurvature = curvature;
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
    notifier.stop();
    odometry.cancel();
    leftDriveTrainVelocityPID.cancel();
    rightDriveTrainVelocityPID.cancel();
    RobotMap.leftDriveLead.set(ControlMode.PercentOutput, 0);
    RobotMap.rightDriveLead.set(ControlMode.PercentOutput, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
