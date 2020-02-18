package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
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
    private int initialValue = 0;
    PIDController drivePid;
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;


    @Override
    public void runOpMode() throws InterruptedException {
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        FRDrive  = hardwareMap.get(DcMotor.class, "front_right");
        FLDrive = hardwareMap.get(DcMotor.class, "front_left");
        BRDrive  = hardwareMap.get(DcMotor.class, "back_right");
        BLDrive  = hardwareMap.get(DcMotor.class, "back_left");


        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);

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



        FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int targetPosition = 5000;
        int robotAngle = 0;

        FRDrive.setTargetPosition(targetPosition);
        FLDrive.setTargetPosition(targetPosition);
        BRDrive.setTargetPosition(targetPosition);
        BLDrive.setTargetPosition(targetPosition);


        double propAccel = 0.2;
        double propDecel = 0.5;
        double accelPow = 2;
        double decelPow = 1.3;

        waitForStart();
        modernRoboticsI2cGyro.resetZAxisIntegrator();

        accelMove(robotAngle,targetPosition, propAccel, propDecel, accelPow, decelPow);

    }

    void accelMove(int desiredAngle, int targetPosition, double propAccel, double propDecel, double accelPow, double decelPow) {
        drivePid.reset();
        drivePid.setSetpoint(desiredAngle);
        drivePid.setInputRange(-359, 359);
        drivePid.setTolerance(1);

        drivePid.enable();

        double accelTarget = targetPosition * propAccel;
        double decelTarget = targetPosition - (targetPosition * propDecel);


        double remainingDist = (accelTarget - FRDrive.getCurrentPosition()) / accelTarget;
        double motorPower = 0;
        double leftPower = 0;
        double rightPower = 0;

        int robotAngle = modernRoboticsI2cGyro.getIntegratedZValue();
        double correction = drivePid.performPID(robotAngle);


        // Acceleration
        while(opModeIsActive() && remainingDist > 0) {
            motorPower = Range.clip(Math.pow(remainingDist, accelPow), 0.1, 1.0);
            if(targetPosition > 0) {
                leftPower = motorPower + correction;
                rightPower = motorPower - correction;
            }  else {
                leftPower = motorPower - correction;
                rightPower = motorPower + correction;
            }

            FRDrive.setPower(1.1 - rightPower);
            FLDrive.setPower(1.1 - leftPower);
            BRDrive.setPower(1.1 - rightPower);
            BLDrive.setPower(1.1 - leftPower);
            remainingDist = (accelTarget - FRDrive.getCurrentPosition()) / accelTarget;
            telemetry.addLine("Status: Accelerating");
            telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getCurrentPosition(), (float)FLDrive.getCurrentPosition(), (float)BRDrive.getCurrentPosition(), (float)BLDrive.getCurrentPosition());
            telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getPower(), (float)FLDrive.getPower(), (float)BRDrive.getPower(), (float)BLDrive.getPower());
            telemetry.update();
        }

        // Constant Velocity
        while(opModeIsActive() && FRDrive.getCurrentPosition() < decelTarget) {
            FRDrive.setPower(1.1 - motorPower);
            FLDrive.setPower(1.1 - motorPower);
            BRDrive.setPower(1.1 - motorPower);
            BLDrive.setPower(1.1 - motorPower);
            telemetry.addLine("Status: Constant Vel");
            telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getCurrentPosition(), (float)FLDrive.getCurrentPosition(), (float)BRDrive.getCurrentPosition(), (float)BLDrive.getCurrentPosition());
            telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getPower(), (float)FLDrive.getPower(), (float)BRDrive.getPower(), (float)BLDrive.getPower());
            telemetry.update();
        }

        remainingDist = (targetPosition - FRDrive.getCurrentPosition()) / (targetPosition - decelTarget);
        telemetry.addData("remainingDist", remainingDist);
        telemetry.update();

        // Deceleration
        while(opModeIsActive() && remainingDist > 0) {
            motorPower = Range.clip(Math.pow(remainingDist, decelPow), 0.2, 1.0);
            FRDrive.setPower(motorPower);
            FLDrive.setPower(motorPower);
            BRDrive.setPower(motorPower);
            BLDrive.setPower(motorPower);
            remainingDist = (targetPosition - FRDrive.getCurrentPosition()) / (targetPosition - decelTarget);
            telemetry.addLine("Status: Decelerating");
            telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getCurrentPosition(), (float)FLDrive.getCurrentPosition(), (float)BRDrive.getCurrentPosition(), (float)BLDrive.getCurrentPosition());
            telemetry.addData("Power", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getPower(), (float)FLDrive.getPower(), (float)BRDrive.getPower(), (float)BLDrive.getPower());
            telemetry.update();
        }

        telemetry.addLine("Status: Move Complete");
        FRDrive.setPower(0);
        FLDrive.setPower(0);
        BRDrive.setPower(0);
        BLDrive.setPower(0);

        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}

