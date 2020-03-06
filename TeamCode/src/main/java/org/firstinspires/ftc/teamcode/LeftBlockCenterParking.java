package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Write changes made to this code that must be made to other skystone autonomous
//

@Autonomous(name="Left Block Center Parking", group="Linear Opmode")
public class LeftBlockCenterParking extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRDrive = null;
    private DcMotor FLDrive = null;
    private DcMotor BRDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor lift = null;
    private Servo erectus = null;
    private Servo frontGrab = null;
    private Servo rightGrab = null;
    private Servo leftGrab = null;
    private Servo foundation = null;
    private Servo capstone = null;
    private double timeout = 5;
    private int FLPosition = 0;
    private int FRPosition = 0;
    private int BLPosition = 0;
    private int BRPosition = 0;
    private int skystonePlacement = 0;

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
        lift = hardwareMap.get(DcMotor.class, "lift");
        erectus = hardwareMap.get(Servo.class, "erectus");
        frontGrab = hardwareMap.get(Servo.class, "front_grab");
        rightGrab = hardwareMap.get(Servo.class, "right_grab");
        leftGrab = hardwareMap.get(Servo.class, "left_grab");
        foundation = hardwareMap.get(Servo.class, "foundation");
        capstone = hardwareMap.get(Servo.class, "capstone");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);
        erectus.setDirection(Servo.Direction.FORWARD);
        frontGrab.setDirection(Servo.Direction.FORWARD);
        rightGrab.setDirection(Servo.Direction.FORWARD);
        leftGrab.setDirection(Servo.Direction.REVERSE);
        foundation.setDirection(Servo.Direction.REVERSE);
        capstone.setDirection(Servo.Direction.FORWARD);

        ModernRoboticsI2cGyro robotGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

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

        //move away from wall
        masterRobot.gyroStraight(robotAngle, 100, 0.7);
        sleep(100);

        //output detect block data
        telemetry.addData("valLeft", masterRobot.getValLeft());
        telemetry.addData("valRight", masterRobot.getValRight());
        telemetry.addData("valMid", masterRobot.getValMid());
        telemetry.update();

        //identify and strafe in front of skystone
        int strafeDistance;
        double strafePower = 0.7;
        if (masterRobot.getValLeft() == 0) {
            skystonePlacement = 1; // Skystone right
            strafeDistance = 275;
            telemetry.addData("Strafing To Right", 1);
            telemetry.update();
        } else if (masterRobot.getValRight() == 0) {
            skystonePlacement = 3; // Skystone left
            strafeDistance = -750;
            telemetry.addData("Strafing To Left", 1);
            telemetry.update();
        } else {
            skystonePlacement = 2; // Skystone center
            telemetry.addData("Strafing To Center", 1);
            telemetry.update();
            strafeDistance = -225;
        }
        masterRobot.gyroStrafe(robotAngle, strafeDistance, strafePower);
        sleep(100);

        //correct possible strafing error
        telemetry.addData("GyroRotate Strafe Correction", 1);
        telemetry.update();
        masterRobot.gyroRotate(robotAngle);
        sleep(100);

        //move up to block
        masterRobot.gyroStraight(robotAngle, 1600, 0.5);
        sleep(100);

        //grab skystone
        masterRobot.grab();
        sleep(100);
        masterRobot.gyroStraight(-175, -175, 0.7);
        sleep(100);

        //rotate towards skybridge to bring FIRST skystone over
        robotAngle -= 84;
        masterRobot.gyroRotate(robotAngle);
        sleep(100);

        //depending on location of the FIRST skystone, move a certain distance under skybridge towards foundation
        if (skystonePlacement == 1) {
            masterRobot.gyroStraight(robotAngle, 2400, 0.7);
            sleep(100);
        } else if (skystonePlacement == 2) {
            masterRobot.gyroStraight(robotAngle, 3000, 0.7);
            sleep(100);
        } else if (skystonePlacement == 3) {
            masterRobot.gyroStraight(robotAngle, 3470, 0.7);
            sleep(100);
        }

        //rotate before foundation and drop off FIRST skystone
        robotAngle += 84;
        masterRobot.gyroRotate(robotAngle);
        sleep(100);
        masterRobot.gyroStraight(robotAngle, 250, 0.7);
        sleep(100);
        masterRobot.release();
        sleep(100);
        masterRobot.gyroStraight(robotAngle, -250, 0.7);
        sleep(100);

        //rotate to go under skybridge to get SECOND skystone
        robotAngle -= 84;
        masterRobot.gyroRotate(robotAngle);
        sleep(100);

        //move a certain distance under the bridge towards stones to get SECOND skystone
        if (skystonePlacement == 1) {
            masterRobot.gyroStraight(robotAngle, -3650, 0.7);
            sleep(100);
        }
        if (skystonePlacement == 2) {
            masterRobot.gyroStraight(robotAngle, -4300, 0.7);
            sleep(100);
        }
        if (skystonePlacement == 3) {
            masterRobot.gyroStraight(robotAngle, -4285, 0.7);
            sleep(100);
        }

        //rotate towards blocks to get SECOND skystone
        robotAngle += 84;
        masterRobot.gyroRotate(robotAngle);
        sleep(100);

        //grab block
        if (skystonePlacement == 1 || skystonePlacement == 2) {
            masterRobot.gyroStraight(robotAngle, 250, 0.7);
            sleep(100);
            masterRobot.grab();
            sleep(100);
            masterRobot.gyroStraight(robotAngle, -250, 0.7);
        } else {
            leftGrab.setPosition(0);
            masterRobot.gyroStraight(robotAngle, 500, 0.2);
            sleep(100);
            leftGrab.setPosition(1);
            sleep(100);
            masterRobot.gyroStraight(robotAngle, -500, 0.7);
        }
        sleep(100);

        //rotate towards skybridge to bring SECOND skystone over
        robotAngle -= 84;
        masterRobot.gyroRotate(robotAngle);
        sleep(100);

        //depending on location of the SECOND skystone, move a certain distance under skybridge towards foundation
        if (skystonePlacement == 1) {
            masterRobot.gyroStraight(robotAngle, 3700, 0.7);
            sleep(250);
        } else if (skystonePlacement == 2) {
            masterRobot.gyroStraight(robotAngle, 4200, 0.7);
            sleep(250);
        } else if (skystonePlacement == 3) {
            masterRobot.gyroStraight(robotAngle, 4350, 0.7);
            sleep(250);
        }

        //let go of skystone
        if (skystonePlacement == 1 || skystonePlacement == 2) {
            masterRobot.release();
        } else {
            leftGrab.setPosition(0);
        }
        sleep(100);

        //park
        masterRobot.gyroStraight(robotAngle, -800, 0.5);
        sleep(500);
    }
}