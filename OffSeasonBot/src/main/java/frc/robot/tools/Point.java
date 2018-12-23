package frc.robot.tools;

public class Point {
    private double xpos;
    private double ypos;
    private double theta;
    public Point(double x, double y) {
        xpos = x;
        ypos = y;
    }
    public Point(double x, double y, double angle){
        xpos = x;
        ypos = y;
        theta = angle;
    }
    public void setLocation(double x, double y){
        xpos = x;
        ypos = y;
    }
    public double getXPos(){
        return xpos;
    }
    public double getYPos(){
        return ypos;
    }
    public double getTheta(){
        return theta;
    }
    

}