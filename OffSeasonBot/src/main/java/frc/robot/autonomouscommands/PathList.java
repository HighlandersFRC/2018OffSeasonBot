/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomouscommands;

import jaci.pathfinder.Waypoint;

public class PathList {
  public PathSetup complexPath1;
  private Waypoint[] complexPath1points = new Waypoint[] {
    new Waypoint(0, 0, 0),
    new Waypoint(12,2.5, 0)
  };
  public PathSetup complexPath2;
  private Waypoint[] complexPath2points = new Waypoint[] {
    new Waypoint(0, 0, 0),
    new Waypoint(2,-1.5, 90), 
  };
  public PathSetup complexPath3;
  private Waypoint[] complexPath3points = new Waypoint[] {
    new Waypoint(0, 0, 0),
    new Waypoint(2,1.5, 90), 
  };
  private double complexPathVelocity = 10;
  public PathList() {
    
    complexPath1 = new PathSetup(complexPath1points, complexPathVelocity,false);
    complexPath2 = new PathSetup(complexPath2points, complexPathVelocity, false);
    complexPath3 = new PathSetup(complexPath3points, complexPathVelocity, true);
  }
  public void resetAllPaths(){
    complexPath1.resetPath();
    complexPath2.resetPath();
  }
}
 
