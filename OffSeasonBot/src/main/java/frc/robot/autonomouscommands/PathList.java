/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class PathList {
  public PathSetup complexPath1;
  private Waypoint[] complexPath1points = new Waypoint[] {
    new Waypoint(0, 0, 0),
    new Waypoint(15.75,9.75,0)
  };
  private double complexPathVelocity = 6;
  public PathList() {
      complexPath1 = new PathSetup(complexPath1points, complexPathVelocity, false);
  }
  public void resetAllPaths(){
    complexPath1.resetPath();
  }
}
 
