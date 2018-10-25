package frc.robot.teleopcommands;

public class TeleopSuite {
	private ArcadeDrive arcadeDrive;
	private TankDrive driveControl;
	public TeleopSuite() {
		arcadeDrive = new ArcadeDrive();
		driveControl = new TankDrive();
		
	}
	public void startTeleopCommands() {
		//driveControl.start();
		arcadeDrive.start();
	}

}
