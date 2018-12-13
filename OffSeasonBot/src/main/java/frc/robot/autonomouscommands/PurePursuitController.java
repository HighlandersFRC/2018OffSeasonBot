/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;



import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotMap;
import frc.robot.RobotMap;
import frc.robot.tools.Point;
import frc.robot.RobotConfig;
import frc.robot.tools.*;

import jaci.pathfinder.Pathfinder;

public class PurePursuitController extends Command {
  private PathSetup chosenPath;
  private Odometry odometry;
  private int closestSegment;
  private Point lookAheadPoint;
  private Notifier notifier;
  private int startingNumber;
  private double deltaX;
  private double deltaY;
  private double distToPoint;
  private double minDistanceToPoint;
  private Point closestPoint;
  private Point startingPointOfLineSegment;
  private Point endPointOfLineSegment;
  private Point robotPos;
  private double lookAheadDistance;
  private double lookAheadIndexT1;
  private double lookAheadIndexT2;
  private double lastPointIndex;
  private double partialPointIndex;
  private Point lastLookAheadPoint;
  private Vector lineSegVector;
  private Vector robotPosVector;
  private int startingNumberLA;
  private boolean firstLookAheadFound;
  private DriveTrainVelocityPID leftDriveTrainVelocityPID;
  private DriveTrainVelocityPID rightDriveTrainVelocityPID;
  private double desiredRobotCurvature;
  private double kValue;
  private double curveAdjustedVelocity;
  
  
  public PurePursuitController(PathSetup path) {
    chosenPath = path;
    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    odometry = new Odometry(chosenPath.getReversed());
    lookAheadPoint = new Point(12, 0);
    closestPoint = new Point(0,0);
    minDistanceToPoint = 100;
    lastLookAheadPoint = new Point(3,3);
    startingPointOfLineSegment = new Point(0,0);
    endPointOfLineSegment = new Point(0,0);
    robotPos = new Point(0,0);
    lineSegVector = new Vector(0, 0);
    robotPosVector = new Vector(0,0);
    lookAheadDistance = 3;
    startingNumber = 0;
    startingNumberLA = 0;
    kValue = 3;
    leftDriveTrainVelocityPID = new DriveTrainVelocityPID(0, RobotMap.leftDriveLead, 1, 0, 0, 0, 0);
    rightDriveTrainVelocityPID = new DriveTrainVelocityPID(0, RobotMap.rightDriveLead, 1, 0, 0, 0, 0);
    leftDriveTrainVelocityPID.start();
    rightDriveTrainVelocityPID.start();
    odometry.start();
    odometry.zero();
    notifier = new Notifier(new PathRunnable());
    notifier.startPeriodic(0.05);
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
      startingNumber = closestSegment;
      minDistanceToPoint = 100;


      firstLookAheadFound = false;
      for(int i = startingNumberLA; i<chosenPath.getMainPath().length()-1;i++){
        startingPointOfLineSegment.setLocation(chosenPath.getMainPath().get(i).x, chosenPath.getMainPath().get(i).y);
        endPointOfLineSegment.setLocation(chosenPath.getMainPath().get(i+1).x, chosenPath.getMainPath().get(i+1).y);
        robotPos.setLocation(odometry.getX(), odometry.getY());
        lineSegVector.setX(endPointOfLineSegment.getXPos()-startingPointOfLineSegment.getXPos());
        lineSegVector.setY(endPointOfLineSegment.getYpos()-startingPointOfLineSegment.getYpos());
        robotPosVector.setX(startingPointOfLineSegment.getXPos()-robotPos.getXPos());
        robotPosVector.setY(startingPointOfLineSegment.getYpos()-robotPos.getYpos());
        double a = lineSegVector.dot(lineSegVector);
        double b = 2*robotPosVector.dot(lineSegVector);
        double c = robotPosVector.dot(robotPosVector)-lookAheadDistance*lookAheadDistance;
        double discriminant = b*b - 4*a*c;
        if(discriminant<0){
          lookAheadPoint.setLocation(lastLookAheadPoint.getXPos(), lastLookAheadPoint.getYpos()); 
        }
        else{
          discriminant = Math.sqrt(discriminant);
          lookAheadIndexT1 = (-b-discriminant)/(2*a);
          lookAheadIndexT2 = (-b+discriminant)/(2*a);
          if(lookAheadIndexT1>=0&&lookAheadIndexT1<=1){
            partialPointIndex = i+lookAheadIndexT1;
            if(partialPointIndex>lastPointIndex){
              lookAheadPoint.setLocation(startingPointOfLineSegment.getXPos()+ lookAheadIndexT1*lineSegVector.getxVec() , startingPointOfLineSegment.getYpos() + lookAheadIndexT1*lineSegVector.getyVec());
              firstLookAheadFound = true;
            }
          }
          
          else if(lookAheadIndexT2>=0&&lookAheadIndexT2<=1){
            partialPointIndex = i+lookAheadIndexT2;
            if(partialPointIndex>lastPointIndex){
              lookAheadPoint.setLocation(startingPointOfLineSegment.getXPos() + lookAheadIndexT2*lineSegVector.getxVec() , startingPointOfLineSegment.getYpos() + lookAheadIndexT2*lineSegVector.getyVec());
              firstLookAheadFound = true;
              
            }
          }
        }
        if(firstLookAheadFound){
          i = chosenPath.getMainPath().length();
        }
        else if(!firstLookAheadFound && i==chosenPath.getMainPath().length()-1){
          lookAheadPoint.setLocation(lastLookAheadPoint.getXPos(), lastLookAheadPoint.getYpos());
        }
      }
      lastLookAheadPoint.setLocation(lookAheadPoint.getXPos(), lookAheadPoint.getYpos());
      if(partialPointIndex>lastPointIndex){
        lastPointIndex = partialPointIndex;
      }
      startingNumberLA = (int)partialPointIndex;


      findRobotCurvature();
      curveAdjustedVelocity = Math.min(chosenPath.getMainPath().get(closestSegment).velocity, kValue/desiredRobotCurvature);
      setWheelVelocities(curveAdjustedVelocity, desiredRobotCurvature);
    }
  } 
  private void setWheelVelocities(double targetVelocity, double curvature){
    double leftVelocity;
    double rightVelocity;
    double v = targetVelocity;
    double c = curvature;
    leftVelocity = v*((2+(c*RobotConfig.robotBaseDist))/2);
    rightVelocity = v*((2-(c*RobotConfig.robotBaseDist))/2);
    SmartDashboard.putNumber("leftVelocity", leftVelocity);
    SmartDashboard.putNumber("rightVelcotiy", rightVelocity);
    SmartDashboard.putNumber("velocity",targetVelocity);
    SmartDashboard.putNumber("curvature", curvature);
    SmartDashboard.putNumber("closestSegment", closestSegment);
  
    //leftDriveTrainVelocityPID.changeDesiredSpeed(leftVelocity);
    //rightDriveTrainVelocityPID.changeDesiredSpeed(rightVelocity);
    
  }
  private void findRobotCurvature(){
    double a = -Math.tan(Pathfinder.d2r(odometry.gettheta()));
    double b = 1;
    double c = Math.tan(Pathfinder.d2r(odometry.gettheta())) * odometry.getX() - odometry.getY();
    double x = Math.abs( a * lookAheadPoint.getXPos()+ b * lookAheadPoint.getYpos() + c) /Math.sqrt(Math.pow(a, 2)+Math.pow(b, 2));
    double side = Math.signum(Math.sin(Pathfinder.d2r(odometry.gettheta())) * (lookAheadPoint.getXPos()-odometry.getX())-Math.cos(Pathfinder.d2r(odometry.gettheta()))*(lookAheadPoint.getYpos()-odometry.getY())); 
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
    if(chosenPath.getLeftFollower().isFinished()||chosenPath.getRightFollower().isFinished()){
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
  }
}
