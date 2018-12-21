package org.firstinspires.ftc.teamcode.onbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class BotBase  extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = TeamConst.vuforiaKey;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /* Declare OpMode members. */
    protected org.firstinspires.ftc.teamcode.onbot.BotBase.MyBot robot = new org.firstinspires.ftc.teamcode.onbot.BotBase.MyBot();

    protected ElapsedTime runtime = new ElapsedTime();
    static final double     WHEEL_BASE_INCHES       = 12;
    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 2.0;

    static final double     TURN_SPEED              = 0.6;

    class MyBot
    {
        /* Public OpMode members. */
        public DcMotor leftDrive   = null;
        public DcMotor  rightDrive  = null;
        public DcMotor liftDrive = null;
        public Servo markerServo = null;
        public Servo hookServo = null;

        /* local OpMode members. */
        HardwareMap hwMap           =  null;
        private ElapsedTime period  = new ElapsedTime();

        /* Constructor */
        public MyBot(){

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors
            leftDrive  = hwMap.get(DcMotor.class, "left_drive");
            rightDrive = hwMap.get(DcMotor.class, "right_drive");
            liftDrive = hwMap.get(DcMotor.class, "lift_drive");

            markerServo = hwMap.get(Servo.class, "marker_servo");
            hookServo = hwMap.get(Servo.class, "hook_servo");  //Hook server motor

            leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
            liftDrive.setDirection(DcMotor.Direction.FORWARD);

            // Set all motors to zero power
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            liftDrive.setPower(0);



        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    protected void initTensorFlow(){
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }
    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

/*
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d - %7f" ,
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.markerServo.getPosition());
        telemetry.update();
*/
        // Wait for the game to start (driver presses PLAY)
        beforeWaitForStart();
        waitForStart();
        // Do the actual work
        runTasks();


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void dropMarker(){
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < 1.5)){
            robot.markerServo.setPosition(0.5);

        }
    }
    public void hookRelease(){
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < 2.5)){
            robot.hookServo.setPosition(1);

        }
    }

    public void hookLock(){
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < 2.5)){
            robot.hookServo.setPosition(0);

        }
    }


    public void resetMarker(){
        runtime.reset();

        while (opModeIsActive() &&
                (runtime.seconds() < 1.5)){
            robot.markerServo.setPosition(0);

        }
    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void liftDrive(double speed,
                          double liftInches,
                          double timeoutS) {
        int newLiftTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLiftTarget = robot.liftDrive.getCurrentPosition() + (int)(liftInches * (COUNTS_PER_MOTOR_REV/(0.5 *3.14)) );
            robot.liftDrive.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            robot.liftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftDrive.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Lifting to %7d ", newLiftTarget);
                telemetry.addData("Path2",  "Lifting from %7d ",
                        robot.liftDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.liftDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


    protected void runForward(double inches, double timeoutS ){
        encoderDrive(DRIVE_SPEED,  inches,  inches, timeoutS);
    }

    protected void lift(double inches, double timeoutS ){
        liftDrive(0.5,  inches,  timeoutS);
    }

    protected void turnDegrees(double angle, double timeoutS){
        double archLength = 3.1415 * WHEEL_BASE_INCHES * angle /360;
        encoderDrive(TURN_SPEED,   archLength, -1*archLength, 4.0);  // turn left

    }

    protected void stopTensorFlow(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    protected String getGoldLocation(double timeoutS){
        String goldLocation ="Unknown";
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds()< timeoutS) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                goldLocation="Left";
                                telemetry.addData("Gold Mineral Position", "Left");
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                goldLocation="Right";
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                goldLocation="Center";
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
        return goldLocation;
    }
    public void runTasks(){

    }

    public void beforeWaitForStart(){

    }
}
