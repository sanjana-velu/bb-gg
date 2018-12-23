package org.firstinspires.ftc.teamcode.onbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto-FacingDepot", group="Tournament")
public class AutoFacingDepot extends BotBase{

    public void beforeWaitForStart(){
        initTensorFlow();
    }
    public void runTasks(){
        latchDown();
        String goldLocation = getGoldLocation(5);

        if(goldLocation.equals("Left")){
            turnDegrees(-45,8);//Turn to face the gold mineral
            runForward(36,8);
            turnDegrees(86,8);
            runForward(37,8);
            dropMarker();
            resetMarker();
            runForward(-101,8);


        }else if (goldLocation.equals("Right")){
            turnDegrees(45,5);
            runForward(47,5);
            turnDegrees(-86,5);
            runForward(37,5);
            dropMarker();
            resetMarker();
            runForward(101,5);


        }else if (goldLocation.equals("Center")){
            runForward(67,11);//move back into the depot while pushing mineral
            turnDegrees(43,8);// turn to face crater
            dropMarker();//drop the marker
            resetMarker();//reset the arm
            runForward(-103,11);//move back into crater

        }
        else{
            runForward(4, 5); // first move forward a little
            turnDegrees(-45, 6.0); // turn left
            runForward(44, 5); // run forward past the mineral marker
            turnDegrees(101, 6.0); // turn right towards the depot
            runForward(36, 5); // run into depot
            dropMarker();
            resetMarker();
            runForward(-95, 6); // run towards the crater
        }

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
