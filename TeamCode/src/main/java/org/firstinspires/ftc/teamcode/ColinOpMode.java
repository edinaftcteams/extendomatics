/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="ColinOpMode", group="Thingmabobba")
//@Disabled
public class ColinOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardwareClassThingmabobba robot = new RobotHardwareClassThingmabobba();   // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double          clawOffset1      = 0;
    //double          clawOffset2      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.06 ;



    @Override
    public void runOpMode() {
        double left;
        double right;
        double central;
        double mech1;
        double left2;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work heref
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            left  = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            mech1 = gamepad2.left_stick_y * 0.5;
            left2 = gamepad2.right_stick_y;

            //add a multiplication symbol to change speed (* 0.5)
            //alternative option - allows for gradient, but difficult to strafe and drive at the same
            central = gamepad1.left_trigger;

            //disadvantage - no power gradient
            if(gamepad1.left_bumper) {
                central = 0.5;
            } else if(gamepad1.right_bumper) {
                central = -0.5;
            }

            // Normalize the values so neither exceed +/- 1.0
            /* max = Math.max(Math.abs(left), Math.abs(right));
           if (max > 1.0)
            {
                left /= max;
                right /= max;
            }
*/


            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            robot.mechMotor1.setPower(mech1);
            robot.centralMotor.setPower(central);
            robot.leftClaw.setPower(left2);
            //robot.leftClaw.setPower(Range.clip(-gamepad2.right_stick_y, 1, -1));
            /*if (gamepad2.y) {
                clawOffset2 += CLAW_SPEED;
            }
            else if (gamepad2.x) {
                clawOffset2 -= CLAW_SPEED;
            }*/
            // Move both servos to new position.  Assume servos are mirror image of each other.

            //robot.leftClaw.setPower(robot.MID_SERVO + clawOffset1);
            /*robot.leftClaw2.setPosition(robot.MID_SERVO + clawOffset1);
            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset2);
*/




            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            //left  = gamepad2.left_stick_y;
          /*  right = gamepad2.left_stick_y;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                //left /= max;
                right /= max;
            }

            //robot.leftMotor2.setPower(left);
            robot.rightMotor2.setPower(right);

*/



            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset1);
            //telemetry.addData("claw",  "Offset = %.2f", clawOffset2);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}




