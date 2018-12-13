package frc.robot.autonomouscommands;

public class AutoSuite {
    private ComplexPath complexPath;
    private PurePursuitController purePursuitController;
    public AutoSuite() {
        complexPath = new ComplexPath();
    }
    public void startAutoCommands() {
        complexPath.start();	
    }

}