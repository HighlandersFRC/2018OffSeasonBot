/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.sensors.VisionCamera;
import frc.robot.tools.Point;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class GetToTarget extends Command {
  private double complexPathVelocity = 2;
  private Waypoint[] pathPoints;
  private PathSetup pathToTarget;
  private PurePursuitController controller;
  private VisionCamera camera;
  private Notifier cameraNotifier;
  private Notifier targetNotifier;
  private boolean isAtTarget;
  private Point robotPoint;
  private double startingAngle;
  private double acceptableError;

  private boolean isTargetFound = false;
  
  public GetToTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  @Override
  protected void initialize() {
   // camera = new VisionCamera(RobotMap.jevois1,5.5);
   // camera.start();
    //if(camera.getDistance() <1){
      //acceptableError = 0.02
    //}
    acceptableError = 0.02;
    isTargetFound = false;
    isAtTarget = false;
    cameraNotifier = new Notifier(new CameraRunnable());
    targetNotifier = new Notifier(new FindTargetRunnable());
    startingAngle = RobotMap.navx.getAngle();
    targetNotifier.startPeriodic(0.05);
  }
  private class CameraRunnable implements Runnable{
    public void run(){
      targetNotifier.stop();
      if(controller.isFinished()){
        isAtTarget = true;
      }
      if(!isAtTarget){
        //if(camera.hasGoodData()){
          robotPoint.setLocation(controller.getX(), controller.getY());
          controller.setOdometry(robotPoint);
        //}
        /*else{
          System.out.println("badData");
        }*/
      }
      else{
        cameraNotifier.stop();
      }
    }
  }
  private class FindTargetRunnable implements Runnable{
    public void run(){
      if(3>0&&!isTargetFound){
        pathPoints = new Waypoint[] {
          new Waypoint(8.0,5.75,0),
          new Waypoint(0, 0,0)
        };
        pathToTarget = new PathSetup(pathPoints, complexPathVelocity, false);
        robotPoint = new Point(0,0,0);
        controller = new PurePursuitController(pathToTarget, 0.35, 3.75,0.002);
        controller.start();
        isTargetFound = true;
      }
      if(isTargetFound){
        targetNotifier.stop();
        cameraNotifier.startPeriodic(2);
      }
    }
  }
  


  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   // System.out.println(RobotMap.navx.getAngle()-startingAngle);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    isAtTarget = true;
    System.out.println(RobotMap.navx.getAngle());
    cameraNotifier.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
