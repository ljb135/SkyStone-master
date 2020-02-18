package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name= "Multithreading", group="Linear Opmode")
//comment out this line before using
public class Multithreading extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRDrive = null;
    private DcMotor FLDrive = null;
    private DcMotor BRDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor lift = null;


    @Override
    public void runOpMode() throws InterruptedException {

        FRDrive  = hardwareMap.get(DcMotor.class, "front_right");
        FLDrive = hardwareMap.get(DcMotor.class, "front_left");
        BRDrive  = hardwareMap.get(DcMotor.class, "back_right");
        BLDrive  = hardwareMap.get(DcMotor.class, "back_left");
        lift = hardwareMap.get(DcMotor.class, "lift");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);


        FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        final int targetPosition = 5000;
        final int thresholdPosition = 3000;

        Thread thread1 = new Thread(){
            public void run(){
                telemetry.addData("FL Moving",1 );
                telemetry.update();
                lift.setTargetPosition(1500);
                lift.setPower(0.5);
                while(lift.isBusy()) {
                    telemetry.addLine("Lift is elevating");
                    telemetry.update();
                }
                lift.setTargetPosition(0);
                lift.setPower(0.5);
                while(lift.isBusy()) {
                    telemetry.addLine("Lift is lowering");
                    telemetry.update();
                }
            }
        };

        waitForStart();

        FLDrive.setTargetPosition(targetPosition);
        BLDrive.setTargetPosition(targetPosition);
        BRDrive.setTargetPosition(targetPosition);
        FRDrive.setTargetPosition(targetPosition);

        FLDrive.setPower(0.8);
        FRDrive.setPower(0.8);
        BLDrive.setPower(0.8);
        BRDrive.setPower(0.8);

        boolean startThread = false;

        while (opModeIsActive() && (FLDrive.isBusy() && FRDrive.isBusy() && BLDrive.isBusy() && BRDrive.isBusy())) {

            if(!startThread && FLDrive.getCurrentPosition() > thresholdPosition) {
                startThread = true;
                thread1.start();
            }

            telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getCurrentPosition(), (float)FLDrive.getCurrentPosition(), (float)BRDrive.getCurrentPosition(), (float)BLDrive.getCurrentPosition());
            telemetry.addData("Target Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getTargetPosition(), (float)FLDrive.getTargetPosition(), (float)BRDrive.getTargetPosition(), (float)BLDrive.getTargetPosition());
            telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getPower(), (float)FLDrive.getPower(), (float)BRDrive.getPower(), (float)BLDrive.getPower());
            telemetry.update();
        }

        thread1.join();

    }
}

