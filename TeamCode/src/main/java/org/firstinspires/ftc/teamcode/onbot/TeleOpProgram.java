package org.firstinspires.ftc.teamcode.onbot;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOpProgram", group="Tournament")
//@Disabled
public class TeleOpProgram extends BotBase{

    public void runTasks(){
        double left;
        double right;
        double drive;
        double turn;
        double max;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  = -1* gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            this.robot.leftDrive.setPower(left);
            this.robot.rightDrive.setPower(right);

            if (gamepad1.dpad_up){
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < 0.5) {
                    this.robot.liftDrive.setPower(0.2);
                }
                this.robot.liftDrive.setPower(0);
            }
            if (gamepad1.dpad_down ){
                runtime.reset();
                while (opModeIsActive() && runtime.seconds() < 0.5) {
                    this.robot.liftDrive.setPower(-0.2);
                }
                this.robot.liftDrive.setPower(0);
            }
            // Map X button to move the hook to lock position
            if (gamepad1.x)
                hookLock();
            // Map B button to move the hook to open position
            if (gamepad1.b)
                hookRelease();


            sleep(10);
        }
    }
}

