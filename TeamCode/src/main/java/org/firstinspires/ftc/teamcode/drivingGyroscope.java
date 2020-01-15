package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Driving Gyro")
public class drivingGyroscope extends LinearOpMode {

    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private DcMotor FRDrive = null;
    private DcMotor FLDrive = null;
    private DcMotor BRDrive = null;
    private DcMotor BLDrive = null;
    private int initialValue = 0;
    private double timeout = 5;
    PIDController rotationPid;
    PIDController drivePid;


    // A runtime helps provide feedback while calibration is taking place
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        FRDrive  = hardwareMap.get(DcMotor.class, "front_right");
        FLDrive = hardwareMap.get(DcMotor.class, "front_left");
        BRDrive  = hardwareMap.get(DcMotor.class, "back_right");
        BLDrive  = hardwareMap.get(DcMotor.class, "back_left");

        FRDrive.setDirection(DcMotor.Direction.REVERSE);
        FLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);

        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //0.9
        rotationPid = new PIDController(0.0095, 0.00005, 0);
        drivePid = new PIDController(0, 0.00001, 0);


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

        int desiredAngle = 84;

        gyroRotate(desiredAngle, 1.0);
    }

    private void gyroRotate(int desiredAngle, double maxPower) {
        if(opModeIsActive()) {

            rotationPid.reset();
            rotationPid.setSetpoint(desiredAngle);
            rotationPid.setInputRange(-359, 359);
            rotationPid.setTolerance(1);
//            rotationPid.setOutputRange(-maxPower, maxPower);
            rotationPid.enable();
            double motorPower = maxPower;
            while (opModeIsActive() && !rotationPid.onTarget()) {

                motorPower = Range.clip(rotationPid.performPID(modernRoboticsI2cGyro.getIntegratedZValue()), -maxPower, maxPower);

                FLDrive.setPower(-motorPower);
                FRDrive.setPower(motorPower);
                BLDrive.setPower(-motorPower);
                BRDrive.setPower(motorPower);

                telemetry.addData("motorPower", motorPower);
                telemetry.addData("integrated Z", modernRoboticsI2cGyro.getIntegratedZValue());
                telemetry.addData("p term", rotationPid.getError() * rotationPid.getP());
                telemetry.addData("total error", rotationPid.getM_totalError());
                telemetry.addData("i term", rotationPid.getM_totalError() * rotationPid.getI());
                telemetry.addData("d error", rotationPid.getM_D_Error());
                telemetry.addData("d term", rotationPid.getM_D_Error() * 0.001);
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

    private void gyroStraight(int targetPosition, double power) {
        if(opModeIsActive()) {
            drivePid.reset();
            drivePid.setSetpoint(0);
            drivePid.setInputRange(-359, 359);
            drivePid.setTolerance(1);

//            rotationPid.setOutputRange(-maxPower, maxPower);
            drivePid.enable();

            double correction = drivePid.performPID(modernRoboticsI2cGyro.getIntegratedZValue());

            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FRDrive.setTargetPosition(targetPosition);
            BRDrive.setTargetPosition(targetPosition);
            FLDrive.setTargetPosition(targetPosition);
            BLDrive.setTargetPosition(targetPosition);

            runtime.reset();

//            while(opModeIsActive() && (FRDrive.getPower() != power || FLDrive.getPower() != power || BLDrive.getPower() != power || BRDrive.getPower() != power)) {
            FLDrive.setPower(power + correction);
            BLDrive.setPower(power + correction);
            FRDrive.setPower(power - correction);
            BRDrive.setPower(power - correction);


            while (opModeIsActive() && (runtime.seconds() < timeout) && (FLDrive.isBusy() && FRDrive.isBusy() && BLDrive.isBusy() && BRDrive.isBusy())) {
                telemetry.addData("integrated Z", modernRoboticsI2cGyro.getIntegratedZValue());
                telemetry.addData("p term", rotationPid.getError() * rotationPid.getP());
                telemetry.addData("total error", rotationPid.getM_totalError());
                telemetry.addData("i term", rotationPid.getM_totalError() * rotationPid.getI());
                telemetry.update();
            }

            FRDrive.setPower(0);
            FLDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

        }
    }
}
