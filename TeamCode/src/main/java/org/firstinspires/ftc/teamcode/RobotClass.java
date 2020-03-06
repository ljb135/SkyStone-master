package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

public class RobotClass {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor lift;

    private Servo capstone;
    private Servo frontGrab;
    private Servo erectus;
    private Servo foundation;
    private Servo rightGrab;
    private Servo leftGrab;

    private LinearOpMode opMode;
    private ElapsedTime runtime;

    private ModernRoboticsI2cGyro robotGyro;

    private int FLPosition;
    private int FRPosition;
    private int BLPosition;
    private int BRPosition;

    private PIDController rotationPid;
    private PIDController drivePid;
    private PIDController strafePid;

    private int skystonePlacement;

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid;
    private static int valLeft;
    private static int valRight;

    private static float rectHeight;
    private static float rectWidth;

    private static float offsetX; //changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY; //changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = new float[2]; //0 = col, 1 = row
    private static float[] leftPos = new float[2];
    private static float[] rightPos = new float[2];
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows;
    private final int cols;

    OpenCvCamera phoneCam;

    public RobotClass(LinearOpMode OP_MODE, ElapsedTime RUNTIME, int MONITOR_ID, ModernRoboticsI2cGyro ROBOT_GYRO,
                      DcMotor FRONT_RIGHT, DcMotor FRONT_LEFT, DcMotor BACK_RIGHT, DcMotor BACK_LEFT, DcMotor LIFT,
                      Servo CAPSTONE, Servo FRONT_GRAB, Servo ERECTUS,
                      Servo FOUNDATION, Servo RIGHT_GRAB, Servo LEFT_GRAB) {

        frontRight = FRONT_RIGHT;
        frontLeft = FRONT_LEFT;
        backRight = BACK_RIGHT;
        backLeft = BACK_LEFT;
        lift = LIFT;
        capstone = CAPSTONE;
        frontGrab = FRONT_GRAB;
        erectus = ERECTUS;
        foundation = FOUNDATION;
        rightGrab = RIGHT_GRAB;
        leftGrab = LEFT_GRAB;
        opMode = OP_MODE;

        robotGyro = ROBOT_GYRO;

        FLPosition = 0;
        FRPosition = 0;
        BLPosition = 0;
        BRPosition = 0;

        runtime = RUNTIME;

        rotationPid = new PIDController(0.01, 0.0001, 0.05);
        drivePid = new PIDController(0.01, 0, 0);
        strafePid = new PIDController(0.07, 0, 0);

        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, MONITOR_ID);

        skystonePlacement = 0;

        //0 means skystone, 1 means yellow stone
        //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
        valMid = -1;
        valLeft = -1;
        valRight = -1;

        rectHeight = .6f/8f;
        rectWidth = 1.5f/8f;

        offsetX = 0f/8f;
        offsetY = 0f/8f;

        midPos[0] = 4f/8f+offsetX;
        midPos[1] = 4f/8f+offsetY;
        leftPos[0] = 2f/8f+offsetX;
        leftPos[1] = 4f/8f+offsetY;
        rightPos[0] = 6f/8f+offsetX;
        rightPos[1] = 4f/8f+offsetY;

        rows = 640;
        cols = 480;
    }


    public void homeServos() {
        if(opMode.opModeIsActive()) {
            rightGrab.setPosition(1);
            leftGrab.setPosition(1);
            capstone.setPosition(1);
            foundation.setPosition(0.2);
            frontGrab.setPosition(0.85);
            erectus.setPosition(0.2);
        }
    }

    public void clampFoundation() {
        if(opMode.opModeIsActive()) {
            foundation.setPosition(1);
        }
    }

    public void releaseFoundation() {
        if(opMode.opModeIsActive()) {
            foundation.setPosition(0.35);
        }
    }

    public void grab() {
        if(opMode.opModeIsActive()) {
            frontGrab.setPosition(0.85);
            opMode.sleep(100);
            erectus.setPosition(0.9);
            opMode.sleep(250);
            frontGrab.setPosition(0);
        }
    }
    public void release() {
        if(opMode.opModeIsActive()) {
            frontGrab.setPosition(0.85);
            opMode.sleep(100);
            erectus.setPosition(0.25);
            opMode.sleep(100);
            frontGrab.setPosition(0);
        }
    }
    //probably delete when drivePID is tuned
    public void move(int left, int right, double power){
        if(opMode.opModeIsActive()){
            FLPosition += left;
            FRPosition += right;
            BLPosition += left;
            BRPosition += right;

            frontLeft.setTargetPosition(FLPosition);
            frontRight.setTargetPosition(FRPosition);
            backLeft.setTargetPosition(BLPosition);
            backRight.setTargetPosition(BRPosition);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setTargetPosition(FLPosition);
            frontRight.setTargetPosition(FRPosition);
            backLeft.setTargetPosition(BLPosition);
            backRight.setTargetPosition(BRPosition);

            opMode.telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getTargetPosition(), (float)frontLeft.getTargetPosition(), (float)backRight.getTargetPosition(), (float)backLeft.getTargetPosition());

            while(opMode.opModeIsActive() && (frontRight.getPower() != power || frontLeft.getPower() != power || backLeft.getPower() != power || backRight.getPower() != power)) {
                opMode.telemetry.addData("updating power,", 1);
                opMode.telemetry.update();
                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);
            }

            while (opMode.opModeIsActive() && (runtime.seconds() < 10) && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
                opMode.telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getCurrentPosition(), (float)frontLeft.getCurrentPosition(), (float)backRight.getCurrentPosition(), (float)backLeft.getCurrentPosition());
                opMode.telemetry.addData("Target Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getTargetPosition(), (float)frontLeft.getTargetPosition(), (float)backRight.getTargetPosition(), (float)backLeft.getTargetPosition());
                opMode.telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getPower(), (float)frontLeft.getPower(), (float)backRight.getPower(), (float)backLeft.getPower());
                opMode.telemetry.update();
            }

            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void gyroRotate(int desiredAngle) {
        if(opMode.opModeIsActive()) {

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rotationPid.reset();
            rotationPid.setSetpoint(desiredAngle);
            rotationPid.setInputRange(-359, 359);
            rotationPid.setTolerance(5);
            rotationPid.enable();
            boolean onTarget = false;
            double motorPower = 0;
//            .abs(rotationPid.getError()) > 5
            while (opMode.opModeIsActive() && !onTarget) {
                motorPower = rotationPid.performPID(robotGyro.getIntegratedZValue());
                onTarget = Math.abs(rotationPid.getError()) < 2;

                frontLeft.setPower(-motorPower);
                frontRight.setPower(motorPower);
                backLeft.setPower(-motorPower);
                backRight.setPower(motorPower);

                opMode.telemetry.addData("onTarget", onTarget);
                opMode.telemetry.addData("motorPower", motorPower);
                opMode.telemetry.addData("integrated Z", robotGyro.getIntegratedZValue());
                opMode.telemetry.addData("error", rotationPid.getError());
                opMode.telemetry.addData("p term", rotationPid.getError() * rotationPid.getP());
                opMode.telemetry.addData("total error", rotationPid.getM_totalError());
                opMode.telemetry.addData("i term", rotationPid.getM_totalError() * rotationPid.getI());
                opMode.telemetry.addData("d error", rotationPid.getM_D_Error());
                opMode.telemetry.addData("d term", rotationPid.getM_D_Error() * rotationPid.getD());
                opMode.telemetry.update();
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            opMode.telemetry.addData("motorPower", motorPower);
            opMode.telemetry.addData("integrated Z", robotGyro.getIntegratedZValue());
            opMode.telemetry.addData("p term", rotationPid.getError() * rotationPid.getP());
            opMode.telemetry.addData("total error", rotationPid.getM_totalError());
            opMode.telemetry.addData("i term", rotationPid.getM_totalError() * rotationPid.getI());
            opMode.telemetry.addData("d error", rotationPid.getM_D_Error());
            opMode.telemetry.addData("d term", rotationPid.getM_D_Error() * 0.001);
            opMode.telemetry.addData("completed rotation", 1);
            opMode.telemetry.update();
        }
    }
    public void gyroStraight(int desiredAngle, int targetPosition, double power) {
        if(opMode.opModeIsActive()) {
            drivePid.reset();
            drivePid.setSetpoint(desiredAngle);
            drivePid.setInputRange(-359, 359);
            drivePid.setTolerance(1);

//            rotationPid.setOutputRange(-maxPower, maxPower);
            drivePid.enable();


            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontRight.setTargetPosition(targetPosition);
            backRight.setTargetPosition(targetPosition);
            frontLeft.setTargetPosition(targetPosition);
            backLeft.setTargetPosition(targetPosition);

            int robotAngle = robotGyro.getIntegratedZValue();
            double correction = drivePid.performPID(robotAngle);
            double leftPower;
            double rightPower;

            if(targetPosition > 0) {
                leftPower = power - correction;
                rightPower = power + correction;
            }  else {
                leftPower = power + correction;
                rightPower = power - correction;
            }

            runtime.reset();

            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);



            while (opMode.opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
                robotAngle = robotGyro.getIntegratedZValue();
                correction = drivePid.performPID(robotAngle);

                if(targetPosition > 0) {
                    leftPower = power - correction;
                    rightPower = power + correction;
                }  else {
                    leftPower = power + correction;
                    rightPower = power - correction;
                }

                frontLeft.setPower(leftPower);
                backLeft.setPower(leftPower);
                frontRight.setPower(rightPower);
                backRight.setPower(rightPower);

                opMode.telemetry.addData("runtime", runtime.seconds());
                opMode.telemetry.addData("in loop", 1);
                opMode.telemetry.addData("correction", correction);
                opMode.telemetry.addData("leftPower", leftPower);
                opMode.telemetry.addData("rightPower", rightPower);
                opMode.telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getCurrentPosition(), (float)frontLeft.getCurrentPosition(), (float)backRight.getCurrentPosition(), (float)backLeft.getCurrentPosition());
                opMode.telemetry.addData("Target Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getTargetPosition(), (float)frontLeft.getTargetPosition(), (float)backRight.getTargetPosition(), (float)backLeft.getTargetPosition());
                opMode.telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getPower(), (float)frontLeft.getPower(), (float)backRight.getPower(), (float)backLeft.getPower());
                opMode.telemetry.update();
            }

            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        }
    }
    public void gyroStrafe(int desiredAngle, int targetPosition, double power){
        if(opMode.opModeIsActive()) {
            strafePid.reset();
            strafePid.setSetpoint(desiredAngle);
            strafePid.setInputRange(-359, 359);
            strafePid.setTolerance(1);

//            rotationPid.setOutputRange(-maxPower, maxPower);
            strafePid.enable();


            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontRight.setTargetPosition(targetPosition);
            backRight.setTargetPosition(-targetPosition);
            frontLeft.setTargetPosition(-targetPosition);
            backLeft.setTargetPosition(targetPosition);

            int robotAngle = robotGyro.getIntegratedZValue();
            double correction = strafePid.performPID(robotAngle);
            double frontPower = 0;
            double backPower = 0;

            if(targetPosition > 0) {
                frontPower = power + correction;
                backPower = power - correction;
            }  else {
                frontPower = power - correction;
                backPower = power + correction;
            }

            runtime.reset();

            frontLeft.setPower(frontPower);
            backLeft.setPower(backPower);
            frontRight.setPower(frontPower);
            backRight.setPower(backPower);


            // Removed timeout
            while (opMode.opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
                robotAngle = robotGyro.getIntegratedZValue();
                correction = strafePid.performPID(robotAngle);

                if(targetPosition > 0) {
                    frontPower = power + correction;
                    backPower = power - correction;
                }  else {
                    frontPower = power - correction;
                    backPower = power + correction;
                }

                frontLeft.setPower(frontPower);
                backLeft.setPower(backPower);
                frontRight.setPower(frontPower);
                backRight.setPower(backPower);

                opMode.telemetry.addData("runtime", runtime.seconds());
                opMode.telemetry.addData("in loop", 1);
                opMode.telemetry.addData("correction", correction);
                opMode.telemetry.addData("frontPower", frontPower);
                opMode.telemetry.addData("backPower", backPower);
                opMode.telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getCurrentPosition(), (float)frontLeft.getCurrentPosition(), (float)backRight.getCurrentPosition(), (float)backLeft.getCurrentPosition());
                opMode.telemetry.addData("Target Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getTargetPosition(), (float)frontLeft.getTargetPosition(), (float)backRight.getTargetPosition(), (float)backLeft.getTargetPosition());
                opMode.telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getPower(), (float)frontLeft.getPower(), (float)backRight.getPower(), (float)backLeft.getPower());
                opMode.telemetry.update();
            }

            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        }
    }
    public void gyroAccelStraight(int desiredAngle, int targetPosition, double propAccel, double propDecel, double accelPow, double decelPow) {
        drivePid.reset();
        drivePid.setSetpoint(desiredAngle);
        drivePid.setInputRange(-359, 359);
        drivePid.setTolerance(1);

        drivePid.enable();

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setTargetPosition(targetPosition);
        frontLeft.setTargetPosition(targetPosition);
        backRight.setTargetPosition(targetPosition);
        backLeft.setTargetPosition(targetPosition);

        double accelTarget = targetPosition * propAccel;
        double decelTarget = targetPosition - (targetPosition * propDecel);


        double remainingDist = (accelTarget - frontRight.getCurrentPosition()) / accelTarget;
        double motorPower = 0;
        double leftPower = 0;
        double rightPower = 0;

        int robotAngle = robotGyro.getIntegratedZValue();
        double correction = drivePid.performPID(robotAngle);


        // Acceleration
        while(opMode.opModeIsActive() && remainingDist > 0) {
            motorPower = Range.clip(Math.pow(remainingDist, accelPow), 0.1, 1.0);
            robotAngle = robotGyro.getIntegratedZValue();
            correction = drivePid.performPID(robotAngle);
            if(targetPosition > 0) {
                leftPower = motorPower + correction;
                rightPower = motorPower - correction;
            }  else {
                leftPower = motorPower - correction;
                rightPower = motorPower + correction;
            }

            frontRight.setPower(1.1 - rightPower);
            frontLeft.setPower(1.1 - leftPower);
            backRight.setPower(1.1 - rightPower);
            backLeft.setPower(1.1 - leftPower);
            remainingDist = (accelTarget - frontRight.getCurrentPosition()) / accelTarget;
            opMode.telemetry.addLine("Status: Accelerating");
            opMode.telemetry.addData("correction", correction);
            opMode.telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getCurrentPosition(), (float)frontLeft.getCurrentPosition(), (float)backRight.getCurrentPosition(), (float)backLeft.getCurrentPosition());
            opMode.telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getPower(), (float)frontLeft.getPower(), (float)backRight.getPower(), (float)backLeft.getPower());
            opMode.telemetry.update();
        }

        // Constant Velocity
        while(opMode.opModeIsActive() && frontRight.getCurrentPosition() < decelTarget) {
            robotAngle = robotGyro.getIntegratedZValue();
            correction = drivePid.performPID(robotAngle);
            if(targetPosition > 0) {
                leftPower = motorPower + correction;
                rightPower = motorPower - correction;
            }  else {
                leftPower = motorPower - correction;
                rightPower = motorPower + correction;
            }
            frontRight.setPower(1.1 - rightPower);
            frontLeft.setPower(1.1 - leftPower);
            backRight.setPower(1.1 - rightPower);
            backLeft.setPower(1.1 - leftPower);
            opMode.telemetry.addLine("Status: Constant Vel");
            opMode.telemetry.addData("correction", correction);
            opMode.telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getCurrentPosition(), (float)frontLeft.getCurrentPosition(), (float)backRight.getCurrentPosition(), (float)backLeft.getCurrentPosition());
            opMode.telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getPower(), (float)frontLeft.getPower(), (float)backRight.getPower(), (float)backLeft.getPower());
            opMode.telemetry.update();
        }

        remainingDist = (targetPosition - frontRight.getCurrentPosition()) / (targetPosition - decelTarget);
        opMode.telemetry.addData("remainingDist", remainingDist);
        opMode.telemetry.update();

        // Deceleration
        while(opMode.opModeIsActive() && remainingDist > 0) {
            motorPower = Range.clip(Math.pow(remainingDist, decelPow), 0.2, 1.0);
            robotAngle = robotGyro.getIntegratedZValue();
            correction = drivePid.performPID(robotAngle);
            if(targetPosition > 0) {
                leftPower = motorPower - correction;
                rightPower = motorPower + correction;
            }  else {
                leftPower = motorPower +correction;
                rightPower = motorPower - correction;
            }
            frontRight.setPower(rightPower);
            frontLeft.setPower(leftPower);
            backRight.setPower(rightPower);
            backLeft.setPower(leftPower);
            remainingDist = (targetPosition - frontRight.getCurrentPosition()) / (targetPosition - decelTarget);
            opMode.telemetry.addLine("Status: Decelerating");
            opMode.telemetry.addData("correction", correction);
            opMode.telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getCurrentPosition(), (float)frontLeft.getCurrentPosition(), (float)backRight.getCurrentPosition(), (float)backLeft.getCurrentPosition());
            opMode.telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)frontRight.getPower(), (float)frontLeft.getPower(), (float)backRight.getPower(), (float)backLeft.getPower());
            opMode.telemetry.update();
        }

        opMode.telemetry.addLine("Status: Move Complete");
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetGyro() {
        opMode.telemetry.log().clear();
        runtime.reset();
        robotGyro.resetZAxisIntegrator();
        opMode.telemetry.addLine("Gyro Reset");
        opMode.telemetry.update();
    }

    public void stopStrafe(){
        if(opMode.opModeIsActive()) {
            FLPosition = 0;
            FRPosition = 0;
            BLPosition = 0;
            BRPosition = 0;
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void calibrateGyro() {
        opMode.telemetry.log().add("Gyro Calibrating. Do Not Move!");
        robotGyro.calibrate();

        // Wait until the gyro calibration is complete
        runtime.reset();
        while (!opMode.isStopRequested() && robotGyro.isCalibrating()) {
            opMode.telemetry.addData("calibrating", "%s", Math.round(runtime.seconds()) % 2 == 0 ? "|.." : "..|");
            opMode.telemetry.update();
            opMode.sleep(50);
        }

        opMode.telemetry.log().clear();
        opMode.telemetry.log().add("Gyro Calibrated. Press Start.");
        opMode.telemetry.clear();
        opMode.telemetry.update();
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

    public void detectSkystone() {
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
    }
    public int getValLeft() {
        return valLeft;
    }
    public int getValRight() {
        return valRight;
    }
    public int getValMid() {
        return valMid;
    }

}
