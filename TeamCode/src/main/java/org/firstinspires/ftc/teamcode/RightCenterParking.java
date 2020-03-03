package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Right Center Parking", group="Linear Opmode")
public class RightCenterParking extends LinearOpMode {
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
        waitForStart();
        masterRobot.resetGyro();
        masterRobot.homeServos();

        //sleep if team asks us to wait until the end to park
        //sleep(25000);

        frontGrab.setPosition(0); //idk if this is the right value *****check grab/release methods
        sleep(250);

        masterRobot.gyroStraight(robotAngle,1400,0.3);
        sleep(500);

        robotAngle -= 84;
        masterRobot.gyroRotate(robotAngle);
        sleep(500);

        masterRobot.gyroStraight(robotAngle,-1850,0.3);
        sleep(500);
    }
}