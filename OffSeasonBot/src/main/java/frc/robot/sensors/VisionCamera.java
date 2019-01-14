/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import java.text.ParseException;

//import org.json.simple.JSONObject;

//import org.json.simple.JSONObject;
//import org.json.simple.parser.JSONParser;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
//import org.json.simple.parser.*;

public class VisionCamera extends Command { 
  private SerialPort visionCamera;
  private boolean shouldRun;
  private String cameraReadout;
  private Notifier cameraNotifier;
  private Object obj;
 // private JSONObject input;
  private double distance;
  private double angle;
  private double x;
  private double y;
  private String sanitizedReadout;
  private int spacePosition;
  private String distance1;
  private String angle1;
  private String distance2;
  private String angle2;
  public VisionCamera(SerialPort camera) {
    visionCamera = camera;
    
    // Use requires() here t
    // declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    shouldRun = true;
    cameraNotifier = new Notifier(new CameraRunnable());
    cameraNotifier.startPeriodic(0.1);
    visionCamera.setReadBufferSize(1);
  }

  private class CameraRunnable implements Runnable{
    public void run(){
      if(shouldRun){
        visionCamera.flush();
        cameraAlgorithm();
      }
      else{
        cameraNotifier.stop();
      }
      
    }
  }
  private void cameraAlgorithm(){

   cameraReadout = visionCamera.readString();
   if(cameraReadout.length()>2){
    distance1 = cameraReadout.substring(0, cameraReadout.indexOf(" "));
    cameraReadout = cameraReadout.substring(cameraReadout.indexOf(" ")+1);
    angle1 = cameraReadout.substring(0, cameraReadout.indexOf(" "));
    cameraReadout = cameraReadout.substring(cameraReadout.indexOf(" ")+1);
    distance2 = cameraReadout.substring(0, cameraReadout.indexOf(" "));
    cameraReadout = cameraReadout.substring(cameraReadout.indexOf(" ")+1);
    angle2 = cameraReadout.substring(0, cameraReadout.indexOf(" "));
    cameraReadout = cameraReadout.substring(cameraReadout.indexOf(" ")+1);

   }

    SmartDashboard.putString("distance1", distance1);    
    SmartDashboard.putString("distance2", distance2);    
    SmartDashboard.putString("angle1", angle1);    
    SmartDashboard.putString("angle2", angle2);    

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
    return "youhavenofriends";
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }
  public boolean hasGoodData(){
    return false;
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
