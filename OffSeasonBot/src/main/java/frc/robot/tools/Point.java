package frc.robot.tools;

public class Point {
    private double xpos;
    private double ypos;
    public Point(double x, double y) {
        xpos = x;
        ypos = y;
    }
    public void setLocation(double x, double y){
        xpos = x;
        ypos = y;
    }
    public double getXPos(){
        return xpos;
    }
    public double getYpos(){
        return ypos;
    }
    

}