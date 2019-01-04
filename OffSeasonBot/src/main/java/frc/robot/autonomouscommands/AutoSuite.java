package frc.robot.autonomouscommands;



public class AutoSuite {
    private GetToTarget getToTarget;
    public AutoSuite() {
        getToTarget = new GetToTarget();
    }
    public void startAutoCommands() {
        getToTarget.start();
    }
    public void End(){
        getToTarget.cancel();
    }

}