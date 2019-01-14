package frc.robot.autonomouscommands;

import frc.robot.RobotMap;

public class AutoSuite {
    private PurePursuitController fPursuitController = new PurePursuitController(RobotMap.universalPathlist.newPathSetup, 2.4, 2.75, 0.05);
    public AutoSuite() {
    }
    public void startAutoCommands() {
        fPursuitController.start();
    }
    public void endTeleopCommands(){
        fPursuitController.cancel();
    }

}