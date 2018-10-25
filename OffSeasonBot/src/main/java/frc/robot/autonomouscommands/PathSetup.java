package frc.robot.autonomouscommands;

import javax.swing.text.Segment;

import frc.robot.RobotConfig;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class PathSetup {
    private double velocity = 4;

    public PathSetup(){

    }
    public Trajectory generateMainPath(){
        // all units are in feet, cause MURICA!, basically the path calculations are assuming 1/20th of a second between updates, and a max velcoity of 8ft/sec, a max acceleration of 9 feet/sec^2, and a max jerk of feet/sec^3
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_LOW, 0.05,velocity,RobotConfig.maxAcceleration, 75.0);
        Waypoint[] points = new Waypoint[] {
            new Waypoint(0, 0, 0),
            new Waypoint(12.5, -8, 0)
        };
    
      Trajectory trajectory = Pathfinder.generate(points, config);
      return trajectory;

    }
    public DistanceFollower generateLeftPathFollower(){
        //this isn't measured wheel base, instead it is effective wheel base which is found by rotating the robot around x times 
        //then C=2PiR and solve for R or r=C/(20Pi) then double I guess units aren't cleared and I kinda just increased it till it seemed to work
        TankModifier modifier = new TankModifier(generateMainPath()).modify(2.54);
        Trajectory left= modifier.getLeftTrajectory();
        DistanceFollower leftFollower = new DistanceFollower(left);
        // this section of code is to create an distance follower which is basically a fancier version of our PID class and then to 
        //modify that for Tank drive 
        return leftFollower;
    }
    public DistanceFollower generateRightPathFollower(){
        //check comments for generateLeftPathFollower() basically same thing
        TankModifier modifier = new TankModifier(generateMainPath()).modify(2.54);
        Trajectory right= modifier.getRightTrajectory();
        DistanceFollower rightfollower = new DistanceFollower(right);
        //this is a way to print out what pathfinder expects the robot to do and how that is supposed to happen
       /* for(int i = 0; i< right.length();i++){
            jaci.pathfinder.Trajectory.Segment seg = right.get(i);
            System.out.println( "%f,%f,%f,%f,%f,%f,%f,%f\n"+ 
            seg.dt +" dt " + seg.x+" dx "+ seg.y+" dy "+ seg.position+" dpos "+ seg.velocity+" dvel "+
                seg.acceleration+" dacc "+ seg.jerk+" dj "+ seg.heading+" dhead ");

        }*/
    
        return rightfollower;
    }
    public void pathdata(){
    }
    public double getVelocity(){
        return velocity;
    }
       
}


