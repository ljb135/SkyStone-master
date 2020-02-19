package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private RobotClass(LinearOpMode OP_MODE, ElapsedTime RUNTIME, ModernRoboticsI2cGyro ROBOT_GYRO, DcMotor FRONT_RIGHT, DcMotor FRONT_LEFT, DcMotor BACK_RIGHT, DcMotor BACK_LEFT, DcMotor LIFT,
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

        rotationPid = new PIDController(0.01, 0.00007, 0.05);
        drivePid = new PIDController(0.01, 0, 0);
        strafePid = new PIDController(0.005, 0, 0);
    }

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
    public void homeServos(){
        rightGrab.setPosition(1);
        leftGrab.setPosition(1);
        capstone.setPosition(1);
        foundation.setPosition(0.2);
        frontGrab.setPosition(0);
        erectus.setPosition(0.25);
    }
    public void grab(){
        frontGrab.setPosition(0.85);
        opMode.sleep(100);
        erectus.setPosition(0.9);
        opMode.sleep(250);
        frontGrab.setPosition(0);
    }
    public void release(){
        frontGrab.setPosition(0.85);
        opMode.sleep(100);
        erectus.setPosition(0.25);
        opMode.sleep(100);
        frontGrab.setPosition(0);
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

            frontLeft.setTargetPosition(targetPosition);
            frontRight.setTargetPosition(targetPosition);
            backLeft.setTargetPosition(targetPosition);
            backRight.setTargetPosition(targetPosition);

            int robotAngle = robotGyro.getIntegratedZValue();
            double correction = drivePid.performPID(robotAngle);
            double leftPower = power + correction;
            double rightPower = power - correction;



            runtime.reset();

            frontLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backLeft.setPower(leftPower);
            backRight.setPower(rightPower);



            while (opMode.opModeIsActive() && (runtime.seconds() < timeout) && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
                robotAngle = robotGyro.getIntegratedZValue();
                correction = drivePid.performPID(robotAngle);
                leftPower = power + correction;
                rightPower = power - correction;

                frontLeft.setPower(leftPower);
                backLeft.setPower(leftPower);
                frontRight.setPower(rightPower);
                backRight.setPower(rightPower);

                opMode.telemetry.addData("runtime", runtime.seconds());
                opMode.telemetry.addData("in loop", 1);
                opMode.telemetry.addData("correction", correction);
                opMode.telemetry.addData("leftPower", leftPower);
                opMode.telemetry.addData("rightPower", rightPower);
                opMode.telemetry.addData("integrated Z", robotAngle);
                opMode.telemetry.addData("error", drivePid.getError());
                opMode.telemetry.addData("p term", drivePid.getError() * drivePid.getP());
                opMode.telemetry.addData("total error", drivePid.getM_totalError());
                opMode.telemetry.addData("i term", drivePid.getM_totalError() * drivePid.getI());
                opMode.telemetry.update();
            }

            frontRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        }
    }
    private void gyroStrafe(int desiredAngle, int targetPosition, double power){
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



            while (opMode.opModeIsActive() && (runtime.seconds() < timeout) && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
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



}
