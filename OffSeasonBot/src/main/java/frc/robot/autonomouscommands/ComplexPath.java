/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import frc.robot.autonomouscommands.PathList;

public class ComplexPath extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ComplexPath() {
    PathRunner path1 = new PathRunner(RobotMap.universalPathlist.complexPath1);
    PathRunner path2 = new PathRunner(RobotMap.universalPathlist.complexPath2);
    PathRunner path3 = new PathRunner(RobotMap.universalPathlist.complexPath3);
    addSequential(path1);
    addSequential( new CascadingPIDTurn(180));
    addSequential(path2);
   // addSequential(path3);
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
