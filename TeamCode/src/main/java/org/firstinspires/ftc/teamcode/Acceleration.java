package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name= "Acceleration", group="Linear Opmode")
//comment out this line before using
public class Acceleration extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRDrive = null;
    private DcMotor FLDrive = null;
    private DcMotor BRDrive = null;
    private DcMotor BLDrive = null;


    @Override
    public void runOpMode() throws InterruptedException {

        FRDrive  = hardwareMap.get(DcMotor.class, "front_right");
        FLDrive = hardwareMap.get(DcMotor.class, "front_left");
        BRDrive  = hardwareMap.get(DcMotor.class, "back_right");
        BLDrive  = hardwareMap.get(DcMotor.class, "back_left");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);

        FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int targetPosition = 5000;

        FRDrive.setTargetPosition(targetPosition);
        FLDrive.setTargetPosition(targetPosition);
        BRDrive.setTargetPosition(targetPosition);
        BLDrive.setTargetPosition(targetPosition);


        double propAccel = 0.2;
        double propDecel = 0.5;
        double accelTarget = targetPosition * propAccel;
        double decelTarget = targetPosition - (targetPosition * propDecel);

        waitForStart();

        // Acceleration
        double remainingDist = (accelTarget - FRDrive.getCurrentPosition()) / accelTarget;
        double motorPower = Range.clip(Math.pow(remainingDist, 2), 0.1, 1.0);

        while(opModeIsActive() && remainingDist > 0) {
            motorPower = Range.clip(Math.pow(remainingDist, 2), 0.1, 1.0);
            FRDrive.setPower(1.1 - motorPower);
            FLDrive.setPower(1.1 - motorPower);
            BRDrive.setPower(1.1 - motorPower);
            BLDrive.setPower(1.1 - motorPower);
            remainingDist = (accelTarget - FRDrive.getCurrentPosition()) / accelTarget;
            telemetry.addData("accelerating", 1.1 - motorPower);
            telemetry.update();
        }

        // Constant Velocity
        while(opModeIsActive() && FRDrive.getCurrentPosition() < decelTarget) {
            FRDrive.setPower(1.1 - motorPower);
            FLDrive.setPower(1.1 - motorPower);
            BRDrive.setPower(1.1 - motorPower);
            BLDrive.setPower(1.1 - motorPower);
            telemetry.addData("constant vel", 1.1 - motorPower);
            telemetry.update();
        }

        remainingDist = (targetPosition - FRDrive.getCurrentPosition()) / (targetPosition - decelTarget);
        telemetry.addData("remainingDist", remainingDist);
        telemetry.update();

        // Deceleration
        while(opModeIsActive() && remainingDist > 0) {
            motorPower = Range.clip(Math.pow(remainingDist, 2), 0.1, 1.0);
            FRDrive.setPower(motorPower);
            FLDrive.setPower(motorPower);
            BRDrive.setPower(motorPower);
            BLDrive.setPower(motorPower);
            remainingDist = (targetPosition - FRDrive.getCurrentPosition()) / (targetPosition - decelTarget);
            telemetry.addData("decelerating", motorPower);
            telemetry.update();
        }

        FRDrive.setPower(0);
        FLDrive.setPower(0);
        BRDrive.setPower(0);
        BLDrive.setPower(0);
    }

}
