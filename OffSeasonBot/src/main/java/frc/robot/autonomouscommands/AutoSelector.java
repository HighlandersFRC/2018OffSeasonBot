/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector extends Command {
  private SendableChooser autoChooser;
  private String AutoOne;
  private String AutoTwo;
  private String AutoThree;

  public AutoSelector() {
    AutoOne = new String("beginningToHatch1");
    AutoTwo = new String("beginningToHatch2");
    AutoThree = new String("beginningToHatch3");
    autoChooser = new SendableChooser<String>();
    autoChooser.addObject("Auto1", AutoOne);
    autoChooser.addObject("Auto2", AutoTwo);
    autoChooser.addObject("Auto3", AutoThree);
    SmartDashboard.putData("autonomousModeChooser",autoChooser);
   
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println(autoChooser.getSelected());

   
   

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
