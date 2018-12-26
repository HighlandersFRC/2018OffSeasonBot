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
  //remember that for all paths if the first point is at (0,0,0) for some reason the end y value is revesred in the coordinate plane
  //for example for a path from (x,y,h) to (0,0,0) a path that goes from (0,0,0) to (x,y,h) would look the same but for one you would 
  // be decreasing y units on the coordinate plane, while in the other you would be increasing y units
  public PathSetup complexPath1;
  private Waypoint[] complexPath1points = new Waypoint[] {
    new Waypoint(15,5.15,0),
    new Waypoint(0, 0, Pathfinder.d2r(90))
  };
  private double complexPathVelocity = 6;
  public PathList() {
      complexPath1 = new PathSetup(complexPath1points, complexPathVelocity, false);
  }
  public void resetAllPaths(){
    complexPath1.resetPath();
  }
}
 
