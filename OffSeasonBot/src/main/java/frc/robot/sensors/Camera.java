package frc.robot.sensors;
public class Camera{
    private double dist;
    private double angle;
    private double x;
    private double y;
    public Camera(){
        dist = 12;
        angle = 30;

    }
    public double getAngle(){
        return angle;
    }
    public double getDist(){
        return dist;
    }
    public double getX(){
        return Math.cos(angle)*dist;
    }
    public double getY(){
        return Math.cos(angle)*dist;
    }

}