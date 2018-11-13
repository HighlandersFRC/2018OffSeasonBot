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
    new Waypoint(6.5, 0, 0),
    new Waypoint(8.5,4,90),
    new Waypoint(12,8,0)
   
  };
  public PathSetup complexPath2;
  private Waypoint[] complexPath2points = new Waypoint[] {
    new Waypoint(0, 0, 0),
    new Waypoint(12,-8, 0), 
  };
  private double complexPathVelocity = 4;
  public PathList() {
    complexPath2 = new PathSetup(complexPath2points, complexPathVelocity, true);
    complexPath1 = new PathSetup(complexPath1points, complexPathVelocity,false);

  }
  public void resetAllPaths(){
    complexPath1.resetPath();
  }
}
 
