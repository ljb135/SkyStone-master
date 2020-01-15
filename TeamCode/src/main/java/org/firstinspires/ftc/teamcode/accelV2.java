package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name= "AccelerationV2", group="Linear Opmode")
//comment out this line before using
public class accelV2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRDrive = null;
    private DcMotor FLDrive = null;
    private DcMotor BRDrive = null;
    private DcMotor BLDrive = null;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getCurrentPosition(), (float)FLDrive.getCurrentPosition(), (float)BRDrive.getCurrentPosition(), (float)BLDrive.getCurrentPosition());
        telemetry.update();

        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Stage 1", true); //drive up to blocks
        amove(1);
        sleep(250);
    }

    private void amove(double power) {
        if (opModeIsActive()) {


            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FLDrive.setTargetPosition(5000);
//            BLDrive.setTargetPosition(8000);
//            FRDrive.setTargetPosition(8000);
//            BRDrive.setTargetPosition(8000);


            runtime.reset();

            double accelVal = 1000; // Number of ticks to accelerate
            double decelVal = 3000; // Number of ticks to decelerate
            double wheelSpeed = 0;
            telemetry.addData("target", FLDrive.getTargetPosition());

            FLDrive.setPower(0.1);
            FRDrive.setPower(0.1);
            BLDrive.setPower(0.1);
            BRDrive.setPower(0.1);

            sleep(250);
            telemetry.addData("Current Position", FLDrive.getCurrentPosition());

//            FLDrive.setPower(1);
//            FRDrive.setPower(1);
//            BLDrive.setPower(1);
//            BRDrive.setPower(1);
//            sleep(1000);
//            telemetry.addData("switching now", 1);
//            telemetry.update();
//            FLDrive.setPower(0.3);
//            FRDrive.setPower(0.3);
//            BLDrive.setPower(0.3);
//            BRDrive.setPower(0.3);
//            sleep(1000);

            while (opModeIsActive() && (((FLDrive.getTargetPosition() - decelVal) - FLDrive.getCurrentPosition()) > 1)) {
                wheelSpeed = Range.scale(FLDrive.getCurrentPosition(), 0, 1000, 0, 1.0);
                if(wheelSpeed > 1.0) {
                    wheelSpeed = 1.0;
                }
                telemetry.addData("accelerating", 1);
                telemetry.addData("wheelSpeed", "Wheel Speed (%.2f) CurrentPosition (%.2f) Target Position: (%.2f)", (float) wheelSpeed, (float)FLDrive.getCurrentPosition(), (float) FLDrive.getTargetPosition());
                telemetry.update();
                FLDrive.setPower(wheelSpeed);
                BLDrive.setPower(wheelSpeed);
                BRDrive.setPower(wheelSpeed);
                FRDrive.setPower(wheelSpeed);
            }

            while (opModeIsActive() && ((FLDrive.getTargetPosition() - FLDrive.getCurrentPosition()) > 1)) {
                telemetry.addData("decelerating", 1);
                wheelSpeed = Range.scale(FLDrive.getCurrentPosition(), FLDrive.getTargetPosition() - decelVal, FLDrive.getTargetPosition(), 0.7, 0);
                if(wheelSpeed > 1.0) {
                    wheelSpeed = 1.0;
                }
                telemetry.addData("wheelSpeed", "Wheel Speed (%.2f) CurrentPosition (%.2f) Target Position: (%.2f)", (float) wheelSpeed, (float)FLDrive.getCurrentPosition(), (float) FLDrive.getTargetPosition());
                telemetry.update();
                FLDrive.setPower(wheelSpeed);
                BLDrive.setPower(wheelSpeed);
                BRDrive.setPower(wheelSpeed);
                FRDrive.setPower(wheelSpeed);
            }
        }
    }
}