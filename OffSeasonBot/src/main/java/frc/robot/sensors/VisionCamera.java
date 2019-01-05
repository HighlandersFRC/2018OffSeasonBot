/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import java.text.ParseException;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import org.json.simple.JSONObject;
import org.json.simple.parser.*;

public class VisionCamera extends Command { 
  private SerialPort visionCamera;
  private boolean shouldRun;
  private String cameraReadout;
  private Notifier cameraNotifier;
  private Object obj;
  private JSONObject input;
  private double distance;
  private double angle;
  private double x;
  private double y;
  public VisionCamera(SerialPort camera) {
    visionCamera = camera;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
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
    obj = null;
    cameraReadout = visionCamera.readString();
    try{
      obj = new JSONParser().parse(cameraReadout);
      input = (JSONObject) obj;
      distance = (double) input.get("Distance");
      angle = (double) input.get("Angle");
      x = distance*Math.cos(angle);
      y = distance*Math.sin(angle);
    }
    catch (org.json.simple.parser.ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
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
