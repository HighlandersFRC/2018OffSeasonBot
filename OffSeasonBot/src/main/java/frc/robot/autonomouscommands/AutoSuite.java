package frc.robot.autonomouscommands;

public class AutoSuite {
    private ComplexPath complexPath;
    private KfDriveTrainCalculation kfDriveTrainCalculation;
    private CascadingPIDTurn cascadingPIDTurn;
    public AutoSuite() {
        complexPath = new ComplexPath();
        kfDriveTrainCalculation = new KfDriveTrainCalculation();
        cascadingPIDTurn = new CascadingPIDTurn(90);
       
	}
    public void startAutoCommands() {
        complexPath.start();	
       // kfDriveTrainCalculation.start();
       //cascadingPIDTurn.start();
    }

}