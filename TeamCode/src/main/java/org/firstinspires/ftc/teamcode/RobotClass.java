package org.firstinspires.ftc.teamcode;

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

    private int FLPosition;
    private int FRPosition;
    private int BLPosition;
    private int BRPosition;

    private LinearOpMode opMode;

    private ElapsedTime runtime;

    public RobotClass(LinearOpMode OP_MODE, ElapsedTime RUNTIME, DcMotor FRONT_RIGHT, DcMotor FRONT_LEFT, DcMotor BACK_RIGHT, DcMotor BACK_LEFT, DcMotor LIFT,
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

        FLPosition = 0;
        FRPosition = 0;
        BLPosition = 0;
        BRPosition = 0;

        runtime = RUNTIME;
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

            runtime.reset();

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




}
