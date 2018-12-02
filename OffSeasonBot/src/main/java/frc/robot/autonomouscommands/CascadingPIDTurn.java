/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotConfig;
import frc.robot.RobotMap;
import frc.robot.sensors.DriveEncoder;
import frc.robot.sensors.Navx;
import frc.robot.tools.PID;

public class CascadingPIDTurn extends Command {
  private Navx navx;
  private DriveTrainVelocityPID leftDriveTrainVelocityPID;
  private DriveTrainVelocityPID rightDriveTrainVelocityPID;
  private PID turnPID;
  private double desiredAngle;
  private DriveEncoder leftSideDriveEncoder;
  private DriveEncoder rightSideDriveEncoder;
  public CascadingPIDTurn(double Angle) {
    desiredAngle = Angle;
    leftDriveTrainVelocityPID = new DriveTrainVelocityPID(4, RobotMap.leftDriveLead, 0, 0.0587, 0.18, 0.0004, 0.8);
    rightDriveTrainVelocityPID = new DriveTrainVelocityPID(4, RobotMap.rightDriveLead, 0, 0.05701, 0.18, 0.0004, 0.8);
    turnPID =  new PID(0.045,0.000035,0.00006 );
    navx = new Navx(RobotMap.navx);
    leftSideDriveEncoder = new DriveEncoder(RobotMap.leftDriveLead, RobotMap.leftDriveLead.getSelectedSensorPosition(0));
    rightSideDriveEncoder = new DriveEncoder(RobotMap.rightDriveLead, RobotMap.rightDriveLead.getSelectedSensorPosition(0));

    requires(RobotMap.drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    turnPID.setMaxOutput(RobotConfig.maxVelocity);
    turnPID.setMinOutput(-RobotConfig.maxVelocity);
    turnPID.setSetPoint(desiredAngle);
    leftDriveTrainVelocityPID.start();
    rightDriveTrainVelocityPID.start();
    navx.softResetYaw(RobotMap.navx.getYaw());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    turnPID.updatePID(navx.currentAngle());
    leftDriveTrainVelocityPID.changeDesiredSpeed(-turnPID.getResult());
    rightDriveTrainVelocityPID.changeDesiredSpeed(turnPID.getResult());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Math.abs(navx.currentAngle()-desiredAngle)<1){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    leftDriveTrainVelocityPID.cancel();
    rightDriveTrainVelocityPID.cancel();
    RobotMap.leftDriveLead.set(ControlMode.PercentOutput, 0);
    RobotMap.rightDriveLead.set(ControlMode.PercentOutput, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    this.end();
  }
}
