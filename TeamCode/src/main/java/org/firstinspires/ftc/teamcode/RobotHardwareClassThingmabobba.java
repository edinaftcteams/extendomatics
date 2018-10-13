package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by colin on 10/25/2016.
 */

public class RobotHardwareClassThingmabobba
{
    /* Public OpMode members. */
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor mechMotor1;
    public DcMotor centralMotor;
    public CRServo leftClaw;
    /*public Servo rightClaw   = null;
    public Servo leftClaw2 = null;
    public Servo pushServo = null;
    */

    public static final double MID_SERVO       =  0.5 ;
    /*public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;*/

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RobotHardwareClassThingmabobba(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        mechMotor1   = hwMap.dcMotor.get("mech_drive1");
        centralMotor  = hwMap.dcMotor.get("central_drive");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        mechMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        centralMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        mechMotor1.setPower(0);
        centralMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mechMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centralMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftClaw = hwMap.crservo.get("left_hand");
        /*rightClaw = hwMap.servo.get("right_hand");
        leftClaw2 = hwMap.servo.get("left_hand2")
        */
        leftClaw.setPower(0);
        /*rightClaw.setPosition(Servo.MIN_POSITION);
        leftClaw2.setPosition(Servo.MIN_POSITION);
        */

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void directDrive(double xVal, double yVal, double rVal) {
        double leftVal = yVal + rVal;
        double rightVal = yVal - rVal;
        double centerVal = xVal;

        leftVal = Range.clip(leftVal, -1, 1);
        rightVal = Range.clip(rightVal, -1, 1);
        centerVal = Range.clip(centerVal, -1, 1);

        leftMotor.setPower(leftVal);
        rightMotor.setPower(rightVal);
        centralMotor.setPower(centerVal);


    }
}