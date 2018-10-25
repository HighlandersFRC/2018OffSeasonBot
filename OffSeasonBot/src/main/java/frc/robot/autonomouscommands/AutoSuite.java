package frc.robot.autonomouscommands;

public class AutoSuite {
	private PathRunner pathRunner;
	public AutoSuite() {
        pathRunner = new PathRunner();
		
		
	}
	public void startAutoCommands() {
		pathRunner.start();
	}

}