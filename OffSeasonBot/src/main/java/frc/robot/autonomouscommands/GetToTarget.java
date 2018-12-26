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
import frc.robot.sensors.Camera;
import frc.robot.tools.Point;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class GetToTarget extends Command {
  private double complexPathVelocity = 6;
  private Waypoint[] pathPoints;
  private PathSetup pathToTarget;
  private PurePursuitController controller;
  private Camera camera;
  private Notifier cameraNotifier;
  private boolean isAtTarget;
  private Point robotPoint;
  
  public GetToTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    camera = new Camera();
    pathPoints = new Waypoint[] {
    new Waypoint(camera.getX(),camera.getY(),camera.getAngle()),
    new Waypoint(0, 0, RobotMap.navx.getAngle())
    };
    pathToTarget = new PathSetup(pathPoints, complexPathVelocity, false);
    controller = new PurePursuitController(pathToTarget, 1.2, 3.75);
    robotPoint = new Point(0, 0,0);
    cameraNotifier = new Notifier(new CameraRunnable());
    cameraNotifier.startPeriodic(0.5);
    
  }
  private class CameraRunnable implements Runnable{
    public void run(){
      if(isAtTarget){
        robotPoint.setLocation(camera.getX(), camera.getY());
        controller.setOdometry(robotPoint);
      }
      else{
        cameraNotifier.stop();
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
