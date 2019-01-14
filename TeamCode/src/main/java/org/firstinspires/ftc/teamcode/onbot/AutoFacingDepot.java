package org.firstinspires.ftc.teamcode.onbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto-FacingDepot", group="Tournament")
public class AutoFacingDepot extends BotBase{

    public void beforeWaitForStart(){
        initTensorFlow();
    }
    public void runTasks(){
        String goldLocation = getGoldLocation(4);

        latchDown();

        if(goldLocation.equals("Left")){
            turnDegrees(-45,8);//Turn to face the gold mineral
            runForward(31,8);
            turnDegrees(80,8);
            runForward(38,8);
            dropMarker();
            resetMarker();
            turnDegrees(25,8);
            runForward(-90,8);


        }else if (goldLocation.equals("Right")){
            turnDegrees(45,8);//Turn to face the gold mineral
            runForward(30,8);
            turnDegrees(-70,8);
            runForward(38,8);
            dropMarker();
            resetMarker();
            turnDegrees(-28,8);
            runForward(-90,8);


        }else if (goldLocation.equals("Center")){
            runForward(69,11);//move back into the depot while pushing mineral
            turnDegrees(55,8);// turn to face crater
            dropMarker();//drop the marker
            resetMarker();//reset the arm
            runForward(-100,11);//move back into crater

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

        //  latchArmReset();
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
