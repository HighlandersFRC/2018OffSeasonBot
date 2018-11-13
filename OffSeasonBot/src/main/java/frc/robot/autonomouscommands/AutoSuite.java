package frc.robot.autonomouscommands;

public class AutoSuite {
    private ComplexPath complexPath;    
    public AutoSuite() {
        complexPath = new ComplexPath();
       
	}
    public void startAutoCommands() {
        complexPath.start();	
    }

}