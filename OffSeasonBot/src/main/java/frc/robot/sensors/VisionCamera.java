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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private String sanitizedReadout;
  private double endDist;
  private boolean dataGood;
  private boolean firstDataFound;
  public VisionCamera(SerialPort camera, double maxDist) {
    visionCamera = camera;
    endDist = maxDist;
    distString = new String();
    sanitizedReadout = new String();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    shouldRun = true;
    dataGood = true;
    firstDataFound = false;
    cameraNotifier = new Notifier(new CameraRunnable());
    cameraNotifier.startPeriodic(0.05);
    visionCamera.reset();
   
    //this is necessary as setting it up in the init.cfg file wasn't working and maintaining an incorrect exposure
    visionCamera.writeString("setcam absexp 32");
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
    cameraReadout = visionCamera.readString();
    System.out.println(cameraReadout);
    if(cameraReadout.length()>0){
      sanitizedReadout = cameraReadout.replaceAll("[^0-9. ]", "");
      SmartDashboard.putString("camreadout", sanitizedReadout);
    }
    if(sanitizedReadout.contains(" ")){
      distString = sanitizedReadout.substring(0, sanitizedReadout.indexOf(" "));
    }
    SmartDashboard.putString("distance", distString);
    if(!distString.isEmpty()){
      distance = Double.parseDouble(distString);
      firstDataFound = true;
      if(distance<endDist){
        dataGood = false;
      }
      SmartDashboard.putNumber("distnum", distance);
    }
  }
  
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   
  }
  public double getX(){
    return distance;
  }
  public double getY(){
    return 0;
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
    if(!dataGood){
      return true;
    }
    return false;
  }
  public boolean hasGoodData(){
    return dataGood;
  }
  // Called once after isFinished returns true
  @Override
  protected void end() {
    shouldRun = false;
    cameraNotifier.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
