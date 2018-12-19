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
  public PathSetup complexPath2;
  private Waypoint[] complexPath2points = new Waypoint[] {
    new Waypoint(0, 0, 0),
    new Waypoint(12,8.5, Pathfinder.d2r(90)) 
  };
  private double complexPathVelocity = 6;
  public PathList() {
   // complexPath1 = new PathSetup(complexPath1points, complexPathVelocity,false);
    complexPath2 = new PathSetup(complexPath2points, complexPathVelocity, false);
  }
  public void resetAllPaths(){
    complexPath2.resetPath();
  }
}
 
