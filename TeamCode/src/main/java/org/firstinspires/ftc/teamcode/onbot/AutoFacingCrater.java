package org.firstinspires.ftc.teamcode.onbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto-FacingCrater", group="Tournament")
public class AutoFacingCrater extends BotBase{
    public void runTasks(){
        // Step through each leg of the path,
        runForward(20, 5); // move forward 20 in
        turnDegrees(-100, 4.0); // turn left
        runForward(51, 6);
        turnDegrees(-45, 4.0); // turn left
        runForward(38, 5); // run forward into depot
        dropMarker();
        resetMarker();
        runForward(-90, 5); // run backwards into crater


    }
}
