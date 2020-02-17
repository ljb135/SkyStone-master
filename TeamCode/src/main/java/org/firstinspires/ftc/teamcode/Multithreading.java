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


    @Override
    public void runOpMode() throws InterruptedException {

        FRDrive  = hardwareMap.get(DcMotor.class, "front_right");
        FLDrive = hardwareMap.get(DcMotor.class, "front_left");
//        BRDrive  = hardwareMap.get(DcMotor.class, "back_right");
//        BLDrive  = hardwareMap.get(DcMotor.class, "back_left");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
//        BRDrive.setDirection(DcMotor.Direction.REVERSE);
//        BLDrive.setDirection(DcMotor.Direction.FORWARD);

        FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        final double power = 0.6;

        Thread thread1 = new Thread(){
            public void run(){
                telemetry.addData("FR Moving",1 );
                telemetry.update();
                FRDrive.setTargetPosition(2000);
                while(FRDrive.getPower() != power){
                    FRDrive.setPower(power);
                }


            }
        };

        Thread thread2 = new Thread(){
            public void run(){
                telemetry.addData("Before sleep",1 );
                telemetry.update();
                try {
                    sleep(5000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                telemetry.addData("FL Moving",1 );
                telemetry.update();
                FLDrive.setTargetPosition(2000);
                while(FLDrive.getPower() != power){
                    FLDrive.setPower(power);
                }

            }
        };

        thread1.run();
        thread2.run();


    }

}
