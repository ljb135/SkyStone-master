package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name= "MasterClassTesting", group="Testing")
//comment out this line before using
public class MasterClassTesting extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FRDrive = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor FLDrive = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor BRDrive = hardwareMap.get(DcMotor.class, "back_right");
        DcMotor BLDrive = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor lift = hardwareMap.get(DcMotor.class, "lift");
        Servo erectus = hardwareMap.get(Servo.class, "erectus");
        Servo frontGrab = hardwareMap.get(Servo.class, "front_grab");
        Servo rightGrab = hardwareMap.get(Servo.class, "right_grab");
        Servo leftGrab = hardwareMap.get(Servo.class, "left_grab");
        Servo foundation = hardwareMap.get(Servo.class, "foundation");
        Servo capstone = hardwareMap.get(Servo.class, "capstone");
        ModernRoboticsI2cGyro robotGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        erectus.setDirection(Servo.Direction.FORWARD);
        frontGrab.setDirection(Servo.Direction.FORWARD);
        rightGrab.setDirection(Servo.Direction.FORWARD);
        leftGrab.setDirection(Servo.Direction.REVERSE);
        foundation.setDirection(Servo.Direction.REVERSE);
        capstone.setDirection(Servo.Direction.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int robotAngle = 0;

        RobotClass masterRobot = new RobotClass(this, runtime, cameraMonitorViewId, robotGyro,
                                                        FRDrive, FLDrive, BRDrive, BLDrive, lift,
                                                        capstone, frontGrab, erectus,
                                                        foundation, rightGrab, leftGrab);

        masterRobot.calibrateGyro();
        masterRobot.detectSkystone();

        waitForStart();

        masterRobot.resetGyro();
        masterRobot.homeServos();
        sleep(500);

        masterRobot.gyroStraight(robotAngle, 5000, 0.7);
//        sleep(500);
//        robotAngle -= 84;
//        masterRobot.gyroRotate(robotAngle);
//        sleep(500);
//        masterRobot.gyroStrafe(robotAngle, 2000, 0.7);
//        sleep(500);
//        robotAngle += 84;
//        masterRobot.gyroRotate(robotAngle);
//        sleep(500);
//        masterRobot.gyroStraight(robotAngle, 2000,0.7);


//        masterRobot.homeServos();
//
//        masterRobot.stopStrafe();
//        masterRobot.move(500, 500, 0.2);
//        sleep(1000);
//        robotAngle-=84;
//        masterRobot.gyroRotate(robotAngle);
//        sleep(1000);
//        masterRobot.grab();
//        masterRobot.release();
//        sleep(1000);
//        masterRobot.clampFoundation();
//        masterRobot.releaseFoundation();
//        sleep(1000);
//        masterRobot.gyroStraight(robotAngle,2000,0.2);
//        sleep(1000);
//        masterRobot.gyroStrafe(robotAngle,-1000,0.3);


        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}