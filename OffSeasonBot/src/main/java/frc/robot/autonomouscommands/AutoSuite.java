package frc.robot.autonomouscommands;

public class AutoSuite {
    private ComplexPath complexPath;
    private Odometry odometry;
    public AutoSuite() {
        complexPath = new ComplexPath();
        odometry = new Odometry();
	}
    public void startAutoCommands() {
        complexPath.start();	
        odometry.start();
    }

}