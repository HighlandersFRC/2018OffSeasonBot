package frc.robot.autonomouscommands;

import java.awt.Point;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class FindLookAheadSegment {
    private Odometry odometry;
    private PathSetup path;
    private Point E;
    private Point L;
    private Point C;
    private int r;
    private double a;
    private double b;
    private double c;
    private double discriminant;
    private Point lastPoint;
    private int startIndex;
    private double lastPointIndex;
    private double lookAheadIndexT1;
    private double lookAheadIndexT2;
    private Point lookAheadPoint;
    private double partialPointIndex;
    public FindLookAheadSegment(PathSetup currentPath, Odometry currentOdometry, int lookAheadDist, double lastLookAheadPointIndex, Point lastLookAheadPoint) {
        path = currentPath;
        E = new Point(0,0);
        L = new Point(0,0);
        C = new Point(0, 0);
      
        odometry = currentOdometry;
        lastPointIndex = lastLookAheadPointIndex;
        r = lookAheadDist;
        lastPoint = lastLookAheadPoint;
        lookAheadPoint = new Point(0,0);
        startIndex = (int)lastPointIndex;
    }
    public Point FindCurrentLookAheadSegment(){
        for(int i = startIndex;i<path.getMainPath().length()-1;i++){
            Vector2d f;
            Vector2d d;
            E.setLocation(path.getMainPath().get(i).x, path.generateMainPath().get(i).y);
            L.setLocation(path.getMainPath().get(i+1).x,path.generateMainPath().get(i+1).y);
            C.setLocation(odometry.getX(), odometry.getY());
            d = new Vector2d(L.getX()-E.getX(),L.getY()-E.getY());
            f = new Vector2d(E.getX()-C.getX(),E.getY()-C.getY());
            a = d.dot(d);
            b = 2*f.dot(d);
            c = f.dot(f)-Math.pow(r, 2);
            discriminant = Math.pow(b, 2)-4*a*c;
            if(!(discriminant<0)){
                discriminant = Math.sqrt(discriminant);
                lookAheadIndexT1 = (-b-discriminant)/(2*a);
                lookAheadIndexT2 = (-b-discriminant)/(2*a);
                if(lookAheadIndexT1>=0&& lookAheadIndexT1<=1){
                    lookAheadPoint.setLocation(E.getX() + lookAheadIndexT1*d.x, E.getY() + lookAheadIndexT1*d.y);
                    partialPointIndex = i+lookAheadIndexT1;
                    if(partialPointIndex > lastPointIndex){
                        return lookAheadPoint;
                    }
                    else{
                        return lastPoint;
                    }
                }   
                else if(lookAheadIndexT2>=0&& lookAheadIndexT2<=1){
                    lookAheadPoint.setLocation(E.getX() + lookAheadIndexT2*d.x, E.getY() + lookAheadIndexT1*d.y);
                    partialPointIndex = i+lookAheadIndexT1;
                    if(partialPointIndex > lastPointIndex ){
                        return lookAheadPoint;
                    }
                    else{
                        return lastPoint;
                    }
                }
                else{
                    return lastPoint;
                }  
            }
            else{
                return lastPoint;
            }   
        }
        return lastPoint;
    }
    public void updatePoint(double newLastIndex, Point newLastPoint){
        lastPointIndex = newLastIndex;
        lastPoint = newLastPoint;
    }
    public double lookAheadIndex(){
        return partialPointIndex;
    }
}