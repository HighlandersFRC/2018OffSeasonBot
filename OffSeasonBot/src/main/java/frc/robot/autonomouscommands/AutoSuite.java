package frc.robot.autonomouscommands;

public class AutoSuite {
    private ComplexPath complexPath;
  
    //private CascadingPIDTurn turn;
    public AutoSuite() {
        complexPath = new ComplexPath();
        //turn = new CascadingPIDTurn(90);
    }
    public void startAutoCommands() {
        complexPath.start();	
    }
    public void End(){
        complexPath.cancel();
    }

}