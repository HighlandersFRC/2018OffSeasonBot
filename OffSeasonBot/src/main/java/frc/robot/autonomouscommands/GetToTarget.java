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
  private double complexPathVelocity = 1;
  private Waypoint[] pathPoints;
  private PathSetup pathToTarget;
  private PurePursuitController controller;
  private VisionCamera camera;
  private Notifier cameraNotifier;
  private Notifier targetNotifier;
  private boolean isAtTarget;
  private Point robotPoint;
  private boolean isTargetFound = false;
  
  public GetToTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  @Override
  protected void initialize() {
    camera = new VisionCamera(RobotMap.jevois1,5.5);
    camera.start();
    isTargetFound = false;
    isAtTarget = false;
    cameraNotifier = new Notifier(new CameraRunnable());
    targetNotifier = new Notifier(new FindTargetRunnable());
    targetNotifier.startPeriodic(0.05);
  }
  private class CameraRunnable implements Runnable{
    public void run(){
      targetNotifier.stop();
      if(!isAtTarget){
        if(camera.hasGoodData()){
          System.out.println("goodData");
          System.out.println(camera.getDistance());
          robotPoint.setLocation(camera.getDistance(), controller.getY());
          controller.setOdometry(robotPoint);
        }
        else{
          System.out.println("badData");
        }
      }
      else{
        cameraNotifier.stop();
      }
    }
  }
  private class FindTargetRunnable implements Runnable{
    public void run(){
      if(camera.getDistance()>0&&!isTargetFound){
        pathPoints = new Waypoint[] {
          new Waypoint(camera.getDistance()-6.0,camera.getY(),0),
          new Waypoint(0, 0,0)
        };
        pathToTarget = new PathSetup(pathPoints, complexPathVelocity, false);
        robotPoint = new Point(0,0,0);
        controller = new PurePursuitController(pathToTarget, 1.2, 3.75,0.1);
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
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
  if(isTargetFound){
    
  }

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    isAtTarget = true;
    cameraNotifier.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
