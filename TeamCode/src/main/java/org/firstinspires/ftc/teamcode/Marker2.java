package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//Move right, Move left, move forward, drop marker
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

@Autonomous(name="Marker2", group="Autonomous")
//@Disabled
public class Marker2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor HDrive = null;
    private DcMotor Claw = null;
    private CRServo Marker = null;
    //private BNO055IMU gyro;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        HDrive = hardwareMap.dcMotor.get("central_drive");
        Claw = hardwareMap.dcMotor.get("mech_drive1");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        Marker = (CRServo) hardwareMap.crservo.get("left_hand");


        waitForStart();
        runtime.reset();

        Claw.setPower(1);
        sleep(1920);
        Claw.setPower(0);
        sleep(500);
        Claw.setPower(0.5);
        sleep(250);
        HDrive.setPower(-0.5);
        sleep(600);
        leftDrive.setPower(-0.5);
        rightDrive.setPower(-0.5);
        sleep(500);
        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);
        sleep(2000);
        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);
        sleep(1000);
        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);
        sleep(3000);


        Marker.setPower(-0.5);
        Thread.sleep(6000);

        Marker.setPower(0.0);
        Thread.sleep(4000);

        Marker.setPower(0.5);
        Thread.sleep(6000);
        Claw.setPower(-1);
        sleep(2000);

    }
}

