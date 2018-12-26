package frc.robot.autonomouscommands;

import frc.robot.RobotMap;
import frc.robot.tools.Point;

public class AutoSuite {
    private ComplexPath complexPath;
    public AutoSuite() {
        complexPath =  new ComplexPath();
    }
    public void startAutoCommands() {
        complexPath.start();
    }
    public void End(){
       complexPath.cancel();
    }

}