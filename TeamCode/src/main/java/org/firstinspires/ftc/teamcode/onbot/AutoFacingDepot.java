package org.firstinspires.ftc.teamcode.onbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto-FacingDepot", group="Tournament")
public class AutoFacingDepot extends BotBase{

    public void beforeWaitForStart(){
        initTensorFlow();
    }
    public void runTasks(){
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        /**/
        // lift(20.0,2);

        String goldLocation = getGoldLocation(5);

        /* Used in Dayton tournament
        runForward(4, 5); // first move forward a little
        turnDegrees(-45, 6.0); // turn left
        runForward(44, 5); // run forward past the mineral marker
        turnDegrees(101, 6.0); // turn right towards the depot
        runForward(36, 5); // run into depot
        dropMarker();
        resetMarker();
        runForward(-95, 6); // run towards the crater

        */
    }
}
