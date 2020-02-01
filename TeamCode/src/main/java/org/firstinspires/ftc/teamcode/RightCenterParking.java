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
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private int initialValue = 0;
    private int robotAngle = 0;
    PIDController rotationPid;
    PIDController drivePid;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRDrive = null;
    private DcMotor FLDrive = null;
    private DcMotor BRDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor Lift = null;
    private Servo Erectus = null;
    private Servo frontGrab = null;
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
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        FRDrive  = hardwareMap.get(DcMotor.class, "front_right");
        FLDrive = hardwareMap.get(DcMotor.class, "front_left");
        BRDrive  = hardwareMap.get(DcMotor.class, "back_right");
        BLDrive  = hardwareMap.get(DcMotor.class, "back_left");
        Lift  = hardwareMap.get(DcMotor.class, "lift");
        Erectus = hardwareMap.get(Servo.class, "erectus");
        frontGrab = hardwareMap.get(Servo.class, "front_grab");
        foundation = hardwareMap.get(Servo.class, "foundation");
        capstone = hardwareMap.get(Servo.class, "capstone");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        Lift.setDirection(DcMotor.Direction.REVERSE);
        Erectus.setDirection(Servo.Direction.FORWARD);
        frontGrab.setDirection(Servo.Direction.FORWARD);
        foundation.setDirection(Servo.Direction.REVERSE);
        capstone.setDirection(Servo.Direction.FORWARD);

        rotationPid = new PIDController(0.01, 0.00007, 0.05);
        drivePid = new PIDController(0.01, 0, 0);


        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        runtime.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(runtime.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        initialValue = modernRoboticsI2cGyro.getIntegratedZValue();
        telemetry.addData("initial value", initialValue);
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();
        telemetry.log().clear();
        runtime.reset();

        modernRoboticsI2cGyro.resetZAxisIntegrator();

        capstone.setPosition(1);

        //move servos into position
        capstone.setPosition(1);
        frontGrab.setPosition(1);
        sleep(100);
        Erectus.setPosition(.25);
        sleep(100);
        foundation.setPosition(0.35);
        sleep(100);

        telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getCurrentPosition(), (float)FLDrive.getCurrentPosition(), (float)BRDrive.getCurrentPosition(), (float)BLDrive.getCurrentPosition());
        telemetry.update();

        //sleep if team asks us to wait until the end to park
        //sleep(22000);

        //drive up near center
        gyroStraight(robotAngle,-1400, 0.3);
        sleep(500);
        frontGrab.setPosition(0);
        sleep(100);

        robotAngle += 84;
        //rotate right
        gyroRotate(robotAngle);
        sleep(500);


        //drive forward and park
        gyroStraight(robotAngle,-1850,0.3);
        sleep(500);
    }

    private void move(int left, int right, double power){
        if(opModeIsActive()){
            FLPosition += left;
            FRPosition += right;
            BLPosition += left;
            BRPosition += right;
            FLDrive.setTargetPosition(FLPosition);
            FRDrive.setTargetPosition(FRPosition);
            BLDrive.setTargetPosition(BLPosition);
            BRDrive.setTargetPosition(BRPosition);

            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FLDrive.setTargetPosition(FLPosition);
            FRDrive.setTargetPosition(FRPosition);
            BLDrive.setTargetPosition(BLPosition);
            BRDrive.setTargetPosition(BRPosition);

            runtime.reset();

            while(FRDrive.getPower() != power || FLDrive.getPower() != power || BLDrive.getPower() != power || BRDrive.getPower() != power){
                FLDrive.setPower(power);
                FRDrive.setPower(power);
                BLDrive.setPower(power);
                BRDrive.setPower(power);
            }

            while (opModeIsActive() && (runtime.seconds() < timeout) && (FLDrive.isBusy() && FRDrive.isBusy() && BLDrive.isBusy() && BRDrive.isBusy())) {
                telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getCurrentPosition(), (float)FLDrive.getCurrentPosition(), (float)BRDrive.getCurrentPosition(), (float)BLDrive.getCurrentPosition());
                telemetry.addData("Target Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getTargetPosition(), (float)FLDrive.getTargetPosition(), (float)BRDrive.getTargetPosition(), (float)BLDrive.getTargetPosition());
                telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getPower(), (float)FLDrive.getPower(), (float)BRDrive.getPower(), (float)BLDrive.getPower());
                telemetry.update();
            }

            FRDrive.setPower(0);
            FLDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    private void gyroRotate(int desiredAngle) {
        if(opModeIsActive()) {

            FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rotationPid.reset();
            rotationPid.setSetpoint(desiredAngle);
            rotationPid.setInputRange(-359, 359);
            rotationPid.setTolerance(5);
            rotationPid.enable();
            boolean onTarget = false;
            double motorPower = 0;
//            .abs(rotationPid.getError()) > 5
            while (opModeIsActive() && !onTarget) {
                motorPower = rotationPid.performPID(modernRoboticsI2cGyro.getIntegratedZValue());
                onTarget = Math.abs(rotationPid.getError()) < 2;

                FLDrive.setPower(-motorPower);
                FRDrive.setPower(motorPower);
                BLDrive.setPower(-motorPower);
                BRDrive.setPower(motorPower);

                telemetry.addData("onTarget", onTarget);
                telemetry.addData("motorPower", motorPower);
                telemetry.addData("integrated Z", modernRoboticsI2cGyro.getIntegratedZValue());
                telemetry.addData("error", rotationPid.getError());
                telemetry.addData("p term", rotationPid.getError() * rotationPid.getP());
                telemetry.addData("total error", rotationPid.getM_totalError());
                telemetry.addData("i term", rotationPid.getM_totalError() * rotationPid.getI());
                telemetry.addData("d error", rotationPid.getM_D_Error());
                telemetry.addData("d term", rotationPid.getM_D_Error() * rotationPid.getD());
                telemetry.update();
            }

            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);
            telemetry.addData("motorPower", motorPower);
            telemetry.addData("integrated Z", modernRoboticsI2cGyro.getIntegratedZValue());
            telemetry.addData("p term", rotationPid.getError() * rotationPid.getP());
            telemetry.addData("total error", rotationPid.getM_totalError());
            telemetry.addData("i term", rotationPid.getM_totalError() * rotationPid.getI());
            telemetry.addData("d error", rotationPid.getM_D_Error());
            telemetry.addData("d term", rotationPid.getM_D_Error() * 0.001);
            telemetry.addData("completed rotation", 1);
            telemetry.update();
        }
    }
    private void gyroStraight(int desiredAngle, int targetPosition, double power) {
        if(opModeIsActive()) {
            drivePid.reset();
            drivePid.setSetpoint(desiredAngle);
            drivePid.setInputRange(-359, 359);
            drivePid.setTolerance(1);

//            rotationPid.setOutputRange(-maxPower, maxPower);
            drivePid.enable();


            FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FRDrive.setTargetPosition(targetPosition);
            BRDrive.setTargetPosition(targetPosition);
            FLDrive.setTargetPosition(targetPosition);
            BLDrive.setTargetPosition(targetPosition);

            int robotAngle = modernRoboticsI2cGyro.getIntegratedZValue();
            double correction = drivePid.performPID(robotAngle);
            double leftPower = power + correction;
            double rightPower = power - correction;



            runtime.reset();

            FLDrive.setPower(leftPower);
            BLDrive.setPower(leftPower);
            FRDrive.setPower(rightPower);
            BRDrive.setPower(rightPower);



            while (opModeIsActive() && (runtime.seconds() < timeout) && (FLDrive.isBusy() && FRDrive.isBusy() && BLDrive.isBusy() && BRDrive.isBusy())) {
                robotAngle = modernRoboticsI2cGyro.getIntegratedZValue();
                correction = drivePid.performPID(robotAngle);
                leftPower = power + correction;
                rightPower = power - correction;

                FLDrive.setPower(leftPower);
                BLDrive.setPower(leftPower);
                FRDrive.setPower(rightPower);
                BRDrive.setPower(rightPower);

                telemetry.addData("runtime", runtime.seconds());
                telemetry.addData("in loop", 1);
                telemetry.addData("correction", correction);
                telemetry.addData("leftPower", leftPower);
                telemetry.addData("rightPower", rightPower);
                telemetry.addData("integrated Z", robotAngle);
                telemetry.addData("error", drivePid.getError());
                telemetry.addData("p term", drivePid.getError() * drivePid.getP());
                telemetry.addData("total error", drivePid.getM_totalError());
                telemetry.addData("i term", drivePid.getM_totalError() * drivePid.getI());
                telemetry.update();
            }

            FRDrive.setPower(0);
            FLDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

        }
    }
}