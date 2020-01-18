package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;



@Autonomous(name= "Left Block Center Parking", group="Linear Opmode")
//comment out this line before using
public class LeftBlockCenterParking extends LinearOpMode {
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private int initialValue = 0;
    private int robotAngle = 0;
    PIDController rotationPid;
    PIDController drivePid;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRDrive = null;
    private DcMotor FLDrive = null;
    private DcMotor BRDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor Lift = null;
    private Servo Erectus = null;
    private Servo frontGrab = null;
    private Servo foundation = null;
    private double timeout = 5;
    private int FLPosition = 0;
    private int FRPosition = 0;
    private int BLPosition = 0;
    private int BRPosition = 0;
    private Servo capstone = null;
    private int distance = 0;
    private int skystonePlacement = 0;

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        FRDrive  = hardwareMap.get(DcMotor.class, "front_right");
        FLDrive = hardwareMap.get(DcMotor.class, "front_left");
        BRDrive  = hardwareMap.get(DcMotor.class, "back_right");
        BLDrive  = hardwareMap.get(DcMotor.class, "back_left");
        Lift  = hardwareMap.get(DcMotor.class, "lift");
        Erectus = hardwareMap.get(Servo.class, "erectus");
        frontGrab = hardwareMap.get(Servo.class, "front_grab");
        foundation = hardwareMap.get(Servo.class, "foundation");
        capstone = hardwareMap.get(Servo.class, "capstone");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Erectus.setDirection(Servo.Direction.FORWARD);
        frontGrab.setDirection(Servo.Direction.FORWARD);
        foundation.setDirection(Servo.Direction.REVERSE);
        capstone.setDirection(Servo.Direction.FORWARD);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        rotationPid = new PIDController(0.01, 0.00007, 0.05);
        drivePid = new PIDController(0.01, 0, 0);


        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        runtime.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(runtime.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        initialValue = modernRoboticsI2cGyro.getIntegratedZValue();
        telemetry.addData("initial value", initialValue);
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();
        telemetry.log().clear();
        runtime.reset();

        capstone.setPosition(0.8);
        foundation.setPosition(1);
        frontGrab.setPosition(1);
        Erectus.setPosition(1);


        telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
        telemetry.addData("Height", rows);
        telemetry.addData("Width", cols);

        telemetry.update();
        sleep(100);

        //move away from wall
        move(100,100,0.3);
        sleep(100);

        int strafeDistance = 75;
        double strafePower = 0.3;
        telemetry.addData("valLeft", valLeft);
        telemetry.update();
        if(valLeft == 0){
            skystonePlacement = 1; // Skystone right
            strafeDistance = 550;
            telemetry.addData("strafingRight", 1);
            telemetry.update();
            strafe(strafeDistance , strafePower);
        } else if(valRight == 0){
            skystonePlacement = 3; // Skystone left
            strafeDistance = -500;
            telemetry.addData("strafingLeft", 1);
            telemetry.update();
            strafe(strafeDistance , strafePower);
        } else{
            telemetry.addData("strafingCenter", 1);
            telemetry.update();
            skystonePlacement = 2; // Skystone center
        }

        stopStrafe();
        sleep(250);
        telemetry.addData("gyroRotate", 1);
        telemetry.update();
        gyroRotate(robotAngle);
        //move up to block
        gyroStraight(robotAngle, 1600,0.4);
        stopStrafe();
        sleep(100);

        //grab block
        frontGrab.setPosition(0.85);
        Erectus.setPosition(0.6);
        sleep(250);
        frontGrab.setPosition(0);
        sleep(250);

        //move back
        move(-100,-100,0.3);
        sleep(250);

        //rotate towards the bridge
        robotAngle -= 84;
        gyroRotate(robotAngle);
        sleep(250);

        //depending on location of the skystone, move a certain distance under the bridge
        if(skystonePlacement == 1){
            gyroStraight(robotAngle,2500,0.5);
            sleep(250);
        }
        else if(skystonePlacement == 2){
            gyroStraight(robotAngle,3050,0.5);
            sleep(250);
        }
        else if(skystonePlacement == 3){
            gyroStraight(robotAngle,3500,0.5);
            sleep(250);
        }

        stopStrafe();

        //rotate before foundation and move forward to drop off block
        robotAngle += 84;
        gyroRotate(robotAngle);
        sleep(250);

        stopStrafe();

        move(200,200, 0.3);
        sleep(100);

        //let go of block
        frontGrab.setPosition(0.85);
        sleep(100);
        Erectus.setPosition(0.6);
        sleep(100);

        move(-200,-200, 0.3);
        sleep(100);

        //rotate to go under bridge
        robotAngle -= 84;
        gyroRotate(robotAngle);
        sleep(100);

        frontGrab.setPosition(0);
        sleep(100);

        stopStrafe();

        //depending on location of the skystone, move a certain distance under the bridge
		if(skystonePlacement == 1){
			gyroStraight(robotAngle,-3850,0.5);
			sleep(250);
		}
		if(skystonePlacement == 2){
			gyroStraight(robotAngle,-4300,0.5);
			sleep(250);
		}
		if(skystonePlacement == 3){
			gyroStraight(robotAngle,-4400,0.5);
			sleep(250);
		}

		frontGrab.setPosition(0.85);

		stopStrafe();
        robotAngle+=84;
		//rotate towards block
		gyroRotate(robotAngle);
		sleep(100);
        stopStrafe();
		move(300,300,0.4);
        sleep(100);

		//grab block
		Erectus.setPosition(0.6);
		sleep(250);
		frontGrab.setPosition(0);
		sleep(250);
		//move back
		move(-275,-275,0.4);
		sleep(250);
		robotAngle-=84;
		gyroRotate(robotAngle);

		//depending on location of the skystone, move a certain distance under the bridge
        if(skystonePlacement == 1){
            gyroStraight(robotAngle,3600,0.5);
            sleep(250);
        }
        else if(skystonePlacement == 2){
            gyroStraight(robotAngle,3850,0.5);
            sleep(250);
        }
        else if(skystonePlacement == 3){
            gyroStraight(robotAngle,4250,0.5);
            sleep(250);
        }

        //let go of block
        frontGrab.setPosition(0.85);
        sleep(250);
        Erectus.setPosition(0.6);
        sleep(100);

        //park
        gyroStraight(robotAngle, -1000, 0.4);
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
    private void move(int left, int right, double power){
        if(opModeIsActive()){
            FLPosition += left;
            FRPosition += right;
            BLPosition += left;
            BRPosition += right;
            FLDrive.setTargetPosition(FLPosition);
            FRDrive.setTargetPosition(FRPosition);
            BLDrive.setTargetPosition(BLPosition);
            BRDrive.setTargetPosition(BRPosition);

            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FLDrive.setTargetPosition(FLPosition);
            FRDrive.setTargetPosition(FRPosition);
            BLDrive.setTargetPosition(BLPosition);
            BRDrive.setTargetPosition(BRPosition);

            telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getTargetPosition(), (float)FLDrive.getTargetPosition(), (float)BRDrive.getTargetPosition(), (float)BLDrive.getTargetPosition());


            runtime.reset();

            while(opModeIsActive() && (FRDrive.getPower() != power || FLDrive.getPower() != power || BLDrive.getPower() != power || BRDrive.getPower() != power)) {
                telemetry.addData("updating power,", 1);
                telemetry.update();
                FLDrive.setPower(power);
                FRDrive.setPower(power);
                BLDrive.setPower(power);
                BRDrive.setPower(power);
            }

            while (opModeIsActive() && (runtime.seconds() < timeout) && (FLDrive.isBusy() && FRDrive.isBusy() && BLDrive.isBusy() && BRDrive.isBusy())) {
                telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getCurrentPosition(), (float)FLDrive.getCurrentPosition(), (float)BRDrive.getCurrentPosition(), (float)BLDrive.getCurrentPosition());
                telemetry.addData("Target Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getTargetPosition(), (float)FLDrive.getTargetPosition(), (float)BRDrive.getTargetPosition(), (float)BLDrive.getTargetPosition());
                telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getPower(), (float)FLDrive.getPower(), (float)BRDrive.getPower(), (float)BLDrive.getPower());
                telemetry.update();
            }

            FRDrive.setPower(0);
            FLDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    private void strafe(int distance, double power){
        if(opModeIsActive()){
            FLPosition -= distance;
            FRPosition += distance;
            BLPosition += distance;
            BRPosition -= distance;
            FLDrive.setTargetPosition(FLPosition);
            FRDrive.setTargetPosition(FRPosition);
            BLDrive.setTargetPosition(BLPosition);
            BRDrive.setTargetPosition(BRPosition);

            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FLDrive.setTargetPosition(FLPosition);
            FRDrive.setTargetPosition(FRPosition);
            BLDrive.setTargetPosition(BLPosition);
            BRDrive.setTargetPosition(BRPosition);

            runtime.reset();

            while(FRDrive.getPower() != power || FLDrive.getPower() != power || BLDrive.getPower() != power || BRDrive.getPower() != power){
                FLDrive.setPower(power);
                FRDrive.setPower(power);
                BLDrive.setPower(power);
                BRDrive.setPower(power);
            }

            while (opModeIsActive() && (runtime.seconds() < timeout) && (FLDrive.isBusy() && FRDrive.isBusy() && BLDrive.isBusy() && BRDrive.isBusy())) {
                telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getCurrentPosition(), (float)FLDrive.getCurrentPosition(), (float)BRDrive.getCurrentPosition(), (float)BLDrive.getCurrentPosition());
                telemetry.addData("Target Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getTargetPosition(), (float)FLDrive.getTargetPosition(), (float)BRDrive.getTargetPosition(), (float)BLDrive.getTargetPosition());
                telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getPower(), (float)FLDrive.getPower(), (float)BRDrive.getPower(), (float)BLDrive.getPower());
                telemetry.update();
            }

            FRDrive.setPower(0);
            FLDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    private void stopStrafe(){
        FLPosition = 0;
        FRPosition = 0;
        BLPosition = 0;
        BRPosition = 0;
        FRDrive.setPower(0);
        FLDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void gyroRotate(int desiredAngle) {
        if(opModeIsActive()) {

            FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rotationPid.reset();
            rotationPid.setSetpoint(desiredAngle);
            rotationPid.setInputRange(-359, 359);
            rotationPid.setTolerance(5);
            rotationPid.enable();
            boolean onTarget = false;
            double motorPower = 0;
//            .abs(rotationPid.getError()) > 5
            while (opModeIsActive() && !onTarget) {
                motorPower = rotationPid.performPID(modernRoboticsI2cGyro.getIntegratedZValue());
                onTarget = Math.abs(rotationPid.getError()) < 2;

                FLDrive.setPower(-motorPower);
                FRDrive.setPower(motorPower);
                BLDrive.setPower(-motorPower);
                BRDrive.setPower(motorPower);

                telemetry.addData("onTarget", onTarget);
                telemetry.addData("motorPower", motorPower);
                telemetry.addData("integrated Z", modernRoboticsI2cGyro.getIntegratedZValue());
                telemetry.addData("error", rotationPid.getError());
                telemetry.addData("p term", rotationPid.getError() * rotationPid.getP());
                telemetry.addData("total error", rotationPid.getM_totalError());
                telemetry.addData("i term", rotationPid.getM_totalError() * rotationPid.getI());
                telemetry.addData("d error", rotationPid.getM_D_Error());
                telemetry.addData("d term", rotationPid.getM_D_Error() * rotationPid.getD());
                telemetry.update();
            }

            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);
            telemetry.addData("motorPower", motorPower);
            telemetry.addData("integrated Z", modernRoboticsI2cGyro.getIntegratedZValue());
            telemetry.addData("p term", rotationPid.getError() * rotationPid.getP());
            telemetry.addData("total error", rotationPid.getM_totalError());
            telemetry.addData("i term", rotationPid.getM_totalError() * rotationPid.getI());
            telemetry.addData("d error", rotationPid.getM_D_Error());
            telemetry.addData("d term", rotationPid.getM_D_Error() * 0.001);
            telemetry.addData("completed rotation", 1);
            telemetry.update();
        }
    }
    private void gyroStraight(int desiredAngle, int targetPosition, double power) {
        if(opModeIsActive()) {
            drivePid.reset();
            drivePid.setSetpoint(desiredAngle);
            drivePid.setInputRange(-359, 359);
            drivePid.setTolerance(1);

//            rotationPid.setOutputRange(-maxPower, maxPower);
            drivePid.enable();


            FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FRDrive.setTargetPosition(targetPosition);
            BRDrive.setTargetPosition(targetPosition);
            FLDrive.setTargetPosition(targetPosition);
            BLDrive.setTargetPosition(targetPosition);

            int robotAngle = modernRoboticsI2cGyro.getIntegratedZValue();
            double correction = drivePid.performPID(robotAngle);
            double leftPower = power + correction;
            double rightPower = power - correction;

            if(targetPosition > 0) {
                leftPower = power - correction;
                rightPower = power + correction;
            }  else {
                leftPower = power + correction;
                rightPower = power - correction;
            }

            runtime.reset();

            FLDrive.setPower(leftPower);
            BLDrive.setPower(leftPower);
            FRDrive.setPower(rightPower);
            BRDrive.setPower(rightPower);



            while (opModeIsActive() && (runtime.seconds() < timeout) && (FLDrive.isBusy() && FRDrive.isBusy() && BLDrive.isBusy() && BRDrive.isBusy())) {
                robotAngle = modernRoboticsI2cGyro.getIntegratedZValue();
                correction = drivePid.performPID(robotAngle);

                if(targetPosition > 0) {
                    leftPower = power - correction;
                    rightPower = power + correction;
                }  else {
                    leftPower = power + correction;
                    rightPower = power - correction;
                }

                FLDrive.setPower(leftPower);
                BLDrive.setPower(leftPower);
                FRDrive.setPower(rightPower);
                BRDrive.setPower(rightPower);

                telemetry.addData("runtime", runtime.seconds());
                telemetry.addData("in loop", 1);
                telemetry.addData("correction", correction);
                telemetry.addData("leftPower", leftPower);
                telemetry.addData("rightPower", rightPower);
                telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getCurrentPosition(), (float)FLDrive.getCurrentPosition(), (float)BRDrive.getCurrentPosition(), (float)BLDrive.getCurrentPosition());
                telemetry.addData("Target Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getTargetPosition(), (float)FLDrive.getTargetPosition(), (float)BRDrive.getTargetPosition(), (float)BLDrive.getTargetPosition());
                telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getPower(), (float)FLDrive.getPower(), (float)BRDrive.getPower(), (float)BLDrive.getPower());
                telemetry.update();
            }

            FRDrive.setPower(0);
            FLDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

        }
    }
}