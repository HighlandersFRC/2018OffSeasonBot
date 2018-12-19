package frc.robot.autonomouscommands;

public class AutoSuite {
    private ComplexPath complexPath;
    private PurePursuitController purePursuitController;
    //private CascadingPIDTurn turn;
    public AutoSuite() {
        complexPath = new ComplexPath();
        //turn = new CascadingPIDTurn(90);
    }
    public void startAutoCommands() {
        complexPath.start();	
        //turn.start();
    }

}