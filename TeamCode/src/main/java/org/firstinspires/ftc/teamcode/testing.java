
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name= "velocityTesting", group="Linear Opmode")
//comment out this line before using
public class testing extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight = null;
    private DcMotor backRight = null;

    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;

    @Override
    public void runOpMode() {


        // get a reference to our ColorSensor object.
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        // Set the LED in the beginning
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.

        while (opModeIsActive()) {
            frontRight.setPower(gamepad1.right_stick_y);
            backRight.setPower(gamepad1.right_stick_y);
            backLeft.setPower(gamepad1.right_stick_y);
            frontLeft.setPower(gamepad1.right_stick_y);

            telemetry.addData("frontRight", calculateVelocity(frontRight));
//            telemetry.addData("backRight", calculateVelocity(backRight));
//            telemetry.addData("frontLeft", calculateVelocity(frontLeft));
//            telemetry.addData("backLeft", calculateVelocity(backLeft));

            telemetry.update();

        }
    }

    double calculateVelocity(DcMotor motor) {
        double currentPosition = motor.getCurrentPosition();
        float velocityThreshold = 250000000; // wait time for velocity sampling, in nS

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.nanoseconds() - velocityThreshold < 0) {}

        double motorVel = (motor.getCurrentPosition() - currentPosition) / velocityThreshold;
        motorVel = motorVel * 41666666.67; // convert from ticks/ns to rpm (6*10^10)/1440


        return motorVel;
    }
}