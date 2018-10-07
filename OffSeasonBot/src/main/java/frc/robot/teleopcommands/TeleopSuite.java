package frc.robot.teleopcommands;

public class TeleopSuite {
	private ArcadeDrive arcadeDrive;
	public TeleopSuite() {
		arcadeDrive = new ArcadeDrive();
		
	}
	public void startTeleopCommands() {
		//driveControl.start();
		arcadeDrive.start();
	}

}
