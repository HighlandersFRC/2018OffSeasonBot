package frc.robot.autonomouscommands;

import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.RobotConfig;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.AccumulatorResult;
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
  private double leftTurnError;
  private double rightTurnError;
  private double leftDesiredAngle;
  private double rightDesiredAngle;
  private double actualAngle;
  private double kt = 0.0;
  private double leftTurn;
  private double rightTurn;
  private DistanceFollower lFollower;
  private DistanceFollower rFollower;
  private PathSetup path;
  private Navx pathNavx;
  private Notifier pathNotifier;
  private DriveEncoder leftEncoder;
  private DriveEncoder rightEncoder;
 
  
  public PathRunner( ) {
    path = new PathSetup();
    lFollower = RobotConfig.rightAutoPath;
    rFollower = RobotConfig.leftAutoPath;
    pathNavx = new Navx(RobotMap.navx);
    leftEncoder = new DriveEncoder(RobotMap.leftDriveLead, 0);
    rightEncoder = new DriveEncoder(RobotMap.rightDriveLead, 0);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
  //FYI A large portion of the explanation can be found by looking at the descriptions on the pathfinder methods
  private class PathRunnable implements Runnable{
   //this is a sperate section of code that runs at a different update rate than the rest, this is necessary to match the dt in this
   //instance 0.05 seconds specified when the path is created
    public void run(){
      leftOutput = lFollower.calculate(leftEncoder.getDistance());
      rightOutput = rFollower.calculate(rightEncoder.getDistance());
      leftDesiredAngle = Pathfinder.d2r(lFollower.getHeading());
      rightDesiredAngle = Pathfinder.d2r(rFollower.getHeading());
      actualAngle =pathNavx.currentYaw();
      leftTurnError = Pathfinder.boundHalfDegrees(leftDesiredAngle - actualAngle);
      rightTurnError = Pathfinder.boundHalfDegrees(rightDesiredAngle - actualAngle);
      rightTurn = kt*rightTurnError;
      leftTurn = kt*leftTurnError;
      //all of the above lines are to calculate your navx input which is scaled and then the velocities are modified accordingly
      RobotMap.leftDriveLead.set(ControlMode.PercentOutput, (leftOutput-leftTurn));
      RobotMap.rightDriveLead.set(ControlMode.PercentOutput,(rightOutput+rightTurn));
      SmartDashboard.putNumber("leftTurnError", leftTurnError);
      SmartDashboard.putNumber("RightTurnError", rightTurnError);
      SmartDashboard.putNumber("leftreportedDistance",leftEncoder.getDistance());
      SmartDashboard.putNumber("rightreportedDistance",rightEncoder.getDistance());

    }

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    path.pathdata();
    //configuring the PIDVA according to jacis specifications, the 1/max velocity is to convert to percent output motor speeds
    lFollower.configurePIDVA(0.1, 0, 0.13, 1/RobotConfig.maxVelocity, 0);
    rFollower.configurePIDVA(0.1, 0, 0.13, 1/RobotConfig.maxVelocity, 0);
    //this is to zero my encoders
    leftEncoder.softReset();
    rightEncoder.softReset();
    kt = (path.getVelocity()/RobotConfig.maxVelocity)*0.000525;
    //this is to have a seperate navx for just the path and make sure that that is zereod
    pathNavx.softResetYaw(RobotMap.mainNavx.currentYaw());
    //below is where the runnable seen above is implemented and setup
    pathNotifier = new Notifier(new PathRunnable());
    pathNotifier.startPeriodic(0.05);
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
    pathNotifier.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
