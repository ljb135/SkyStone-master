/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;


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

@TeleOp(name="Manual Control", group="Iterative Opmode")
public class Manual extends LinearOpMode {

    // Declare OpMode members.
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
    private double FL = 0;
    private double FR = 0;
    private double BL = 0;
    private double BR = 0;
    private boolean sensitive = false;
    private boolean grab = false;
    private boolean drag = false;
    private boolean raise = false;

    @Override
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
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Erectus.setDirection(Servo.Direction.FORWARD);
        frontGrab.setDirection(Servo.Direction.FORWARD);
        foundation.setDirection(Servo.Direction.REVERSE);
        capstone.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        capstone.setPosition(0.8);
        frontGrab.setPosition(0.85);
        sleep(500);
        Erectus.setPosition(0.6);
        foundation.setPosition(0.45);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            if(Lift.getCurrentPosition() > 2680){
                Lift.setPower(Range.clip(-gamepad2.left_stick_y, -0.8, 0));
            }
            else if(Lift.getCurrentPosition() < 80){
                Lift.setPower(Range.clip(-gamepad2.left_stick_y, 0, 1.0));
            } else {
                Lift.setPower(Range.clip(-gamepad2.left_stick_y, -0.8, 1.0));
            }

            double threshold = 0;

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            if(!sensitive && (abs(gamepad1.left_stick_y) >= threshold || abs(gamepad1.left_stick_x) >= threshold)) {
                FR = Range.clip((-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2, -1.0, 1.0);
                FL = Range.clip((-gamepad1.left_stick_y + gamepad1.left_stick_x) / 2, -1.0, 1.0);
                BR = Range.clip((-gamepad1.left_stick_y + gamepad1.left_stick_x) / 2, -1.0, 1.0);
                BL = Range.clip((-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2, -1.0, 1.0);
            }
            else if (abs(gamepad1.left_stick_y) >= threshold || abs(gamepad1.left_stick_x) >= threshold) {
                FR = Range.scale((-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2, -1.0, 1.0, -0.5,0.5);
                FL = Range.scale((-gamepad1.left_stick_y + gamepad1.left_stick_x) / 2, -1.0, 1.0, -0.5,0.5);
                BR = Range.scale((-gamepad1.left_stick_y + gamepad1.left_stick_x) / 2, -1.0, 1.0, -0.5,0.5);
                BL = Range.scale((-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2, -1.0, 1.0, -0.5,0.5);
            }
            if (abs(gamepad1.right_stick_x) > threshold) {
                //rotate
                FR = Range.clip((-gamepad1.right_stick_x) / 2, -0.6, 0.6);
                FL = Range.clip((gamepad1.right_stick_x) / 2, -0.6, 0.6);
                BR = Range.clip((-gamepad1.right_stick_x) / 2, -0.6, 0.6);
                BL = Range.clip((gamepad1.right_stick_x) / 2, -0.6, 0.6);
            }

            FRDrive.setPower(FR);
            FLDrive.setPower(FL);
            BRDrive.setPower(BR);
            BLDrive.setPower(BL);

//            if(gamepad2.dpad_up && Lift.getCurrentPosition() < 2680){
//                Lift.setPower(0.5);
//            }
//            else if(gamepad2.dpad_down && Lift.getCurrentPosition() > 80){
//                Lift.setPower(-0.5);
//            } else {
//                Lift.setPower(0);
//            }



            if(gamepad2.a && !grab){
                frontGrab.setPosition(0);
                grab = true;
                sleep(500);
            }
            else if(gamepad2.a){
                frontGrab.setPosition(0.85);
                grab = false;
                sleep(500);
            }

            if(gamepad2.b && !drag){
                foundation.setPosition(0.8);
                drag = true;
                sleep(500);
            }
            else if(gamepad2.b){
                foundation.setPosition(0.45);
                drag = false;
                sleep(500);
            }

            if(gamepad2.y){
                frontGrab.setPosition(0.85);
                sleep(250);
                Erectus.setPosition(0.6);
                raise = false;
                while(Lift.getCurrentPosition() > 80){
                    Lift.setPower(-1);
                }
                Lift.setPower(0);
            }

            if(gamepad2.x && !raise){
                frontGrab.setPosition(0);
                sleep(250);
                Erectus.setPosition(1);
                raise = true;
                sleep(250);
            }
            else if(gamepad2.x){
                frontGrab.setPosition(0.85);
                sleep(250);
                Erectus.setPosition(0.6);
                raise = false;
                sleep(250);
            }

            if(gamepad1.a){
                capstone.setPosition(0);
                sleep(1000);
                capstone.setPosition(0.8);
            }

            if(gamepad1.b){
                sensitive = !sensitive;
                sleep(500);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Controller", "X1: (%.2f) Y1: (%.2f) X2: (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("Lift Value", Lift.getCurrentPosition());
            telemetry.addData("Position", "FR: (%.2f) FL: (%.2f) BR: (%.2f) BL: (%.2f)", (float)FRDrive.getCurrentPosition(), (float)FLDrive.getCurrentPosition(), (float)BRDrive.getCurrentPosition(), (float)BLDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}
