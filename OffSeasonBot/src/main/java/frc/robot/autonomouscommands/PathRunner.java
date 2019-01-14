package frc.robot.autonomouscommands;

import java.awt.geom.Arc2D;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.RobotConfig;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;
import frc.robot.autonomouscommands.PathSetup;
import frc.robot.sensors.DriveEncoder;
import frc.robot.sensors.Navx;

public class PathRunner extends Command {
  private double leftOutput;
  private double rightOutput;
  private double turnError;
  private double desiredAngle;
  private double actualAngle;
  private double kt = 0.0;
  private double turn;
  private DistanceFollower lFollower;
  private DistanceFollower rFollower;
  private PathSetup path;
  private Navx pathNavx;
  private Notifier pathNotifier;
  private DriveEncoder leftEncoder;
  private DriveEncoder rightEncoder;
  private Odometry odometry;
 
  
  public PathRunner(PathSetup chosenpath) {
    path = chosenpath;
    lFollower = path.getLeftFollower();
    rFollower = path.getRightFollower();
    pathNavx = new Navx(RobotMap.navx);
    leftEncoder = new DriveEncoder(RobotMap.leftDriveLead, 0);
    rightEncoder = new DriveEncoder(RobotMap.rightDriveLead, 0);
    odometry = new Odometry(false);
    odometry.start();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
  //FYI A large portion of the explanation can be found by looking at the descriptions on the pathfinder methods
  private class PathRunnable implements Runnable{
   //this is a sperate section of code that runs at a different update rate than the rest, this is necessary to match the dt in this
   //instance 0.05 seconds specified when the path is created
    public void run(){
      if(path.getReversed()){
        leftOutput = lFollower.calculate(-leftEncoder.getDistance());
        rightOutput = rFollower.calculate(-rightEncoder.getDistance());
        desiredAngle = (Pathfinder.boundHalfDegrees(Pathfinder.r2d(lFollower.getHeading())));
        actualAngle = pathNavx.currentReverseYaw();
        turnError = Pathfinder.boundHalfDegrees((desiredAngle - actualAngle));
      }
      else{
        leftOutput = lFollower.calculate(leftEncoder.getDistance());
        rightOutput = rFollower.calculate(rightEncoder.getDistance());
        desiredAngle = (Pathfinder.boundHalfDegrees(Pathfinder.r2d(lFollower.getHeading())));
        actualAngle = pathNavx.currentYaw();
        turnError = Pathfinder.boundHalfDegrees(desiredAngle - actualAngle);
        
      }
       
      turn = kt*turnError;
      //all of the above lines are to calculate your navx input which is scaled and then the velocities are modified accordingly
      if(path.getReversed()){
        RobotMap.leftDriveLead.set(ControlMode.PercentOutput,(-(leftOutput-turn)));
        RobotMap.rightDriveLead.set(ControlMode.PercentOutput,(-(rightOutput+turn)));
      }
      else{
        RobotMap.leftDriveLead.set(ControlMode.PercentOutput, ((leftOutput-turn)));
        RobotMap.rightDriveLead.set(ControlMode.PercentOutput,((rightOutput+turn)));
      }
      SmartDashboard.putNumber("odometryx", odometry.getX());
      SmartDashboard.putNumber("odometryy",odometry.getY());
      SmartDashboard.putNumber("odometrytheta", odometry.gettheta());
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //path.pathdata();
    //configuring the PIDVA according to jacis specifications, the 1/max velocity is to convert to percent output motor speeds
    lFollower.configurePIDVA(1.1, 0, 0.10, 1/RobotConfig.maxVelocity, 0);
    rFollower.configurePIDVA(1.1, 0, 0.10, 1/RobotConfig.maxVelocity, 0);
    //this is to zero my encoders
    leftEncoder.softReset();
    rightEncoder.softReset();
    kt = (path.getVelocity()/RobotConfig.maxVelocity)*0.05;
    //this is to have a seperate navx for just the path and make sure that that is zereod
    pathNavx.softResetYaw();
    //below is where the runnable seen above is implemented and setup
    pathNotifier = new Notifier(new PathRunnable());
    pathNotifier.startPeriodic(0.05);
    odometry.zero();
    if(path.getReversed()){
      odometry.reverseOdometry(true);
    }
    else{
      odometry.reverseOdometry(false);
    }
    SmartDashboard.putNumber("number", path.getMainPath().length());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }
  
  public double currentPathTime(){
    return lFollower.getSegment().dt;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(lFollower.isFinished()&&rFollower.isFinished()){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.leftDriveLead.set(ControlMode.PercentOutput, 0);
    RobotMap.rightDriveLead.set(ControlMode.PercentOutput, 0);
    pathNavx.softResetYaw();
    pathNotifier.stop();
    odometry.zero();
    odometry.cancel();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
