/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;

public class VisionCamera extends Command { 
  private SerialPort visionCamera;
  private double x;
  private double y;
  private volatile double angle;
  private volatile double distance;
  private String cameraReadout;
  private int positionOfSpace;
  private String distString;
  private String angleString;
  private boolean shouldRun;
  private Notifier cameraNotifier;
  public VisionCamera(SerialPort camera) {
    visionCamera = camera;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    shouldRun = true;
    cameraNotifier = new Notifier(new CameraRunnable());
    cameraNotifier.startPeriodic(0.5);
  }

  private class CameraRunnable implements Runnable{
    public void run(){
      if(shouldRun){
        cameraAlgorithm();
      }
      else{
        cameraNotifier.stop();
      }
      
    }
  }
  private void cameraAlgorithm(){
    if(visionCamera.getBytesReceived()>3){
      cameraReadout = visionCamera.readString();
      positionOfSpace = cameraReadout.indexOf(' ');
      distString = cameraReadout.substring(0, positionOfSpace);
      angleString = cameraReadout.substring(positionOfSpace);
      angle = Pathfinder.d2r(Double.parseDouble(angleString));
      distance = Double.parseDouble(distString);
      x = Math.cos(angle)*distance;
      y = Math.sin(angle)*distance;
    }
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   
  }
  public double getX(){
    return x;
  }
  public double getY(){
    return y;
  }
  public double getAngle(){
    return angle;
  }
  public double getDistance(){
    return distance;
  }
  public String getSerialString(){
    return cameraReadout;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    cameraNotifier.stop();
    shouldRun = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
