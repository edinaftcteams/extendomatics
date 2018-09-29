package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="ColinAutoMode", group="RobotHardwareClassThingmabobba")


public class ColinAutoMode extends ColinOpMode {
    RobotHardwareClassThingmabobba         robot   = new RobotHardwareClassThingmabobba();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU gyro;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     FORWARD_SPEED           = 0.25;
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    static final double     OPEN_SPEED              = 0.1;









    @Override
    public void runOpMode() {


       /*
        * Initialize the drive system variables.
        * The init() method of the hardware class does all the work here
        */
        robot.init(hardwareMap);
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "Extendomatics3764.json";
        gyro.initialize(parameters);
        robot.leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        robot.rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();


        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        /*encoderDriveY(DRIVE_SPEED, 17, 17, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDriveY(TURN_SPEED,   4.25, -4.25, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDriveY(DRIVE_SPEED, 4, 4, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        */
        //gyroTurn( TURN_SPEED, 45.0);   // Turn  CCW to -45 Degrees
        gyroDrive(DRIVE_SPEED, 18.0, 0.0);    // Drive FWD 48 inches
        robot.centralMotor.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        gyroDrive(DRIVE_SPEED,12.0, 0.0);    // Drive FWD 48 inches
        /*robot.leftClaw.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/


        //encoderDriveX(DRIVE_SPEED, 10, 1.0);
        //gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second


        telemetry.addData("Path", "Complete");
        telemetry.update();

        sleep(3000);







    }
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftMotor.setPower(leftSpeed);
                robot.rightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftMotor.setPower(leftSpeed);
        robot.rightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getAngularOrientation().firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    /*
    *  Method to perfmorm a relative move, based on encoder counts.
    *  Encoders are not reset as the move is based on the current position.
    *  Move will stop if any of three conditions occur:
    *  1) Move gets to the desired position
    *  2) Move runs out of time
    *  3) Driver stops the opmode running.
    */
    /*public void encoderDriveY(double speed,
                              double yInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;




        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(yInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(yInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




            // reset the timeout time and start motion.
            runtime.reset();
            robot.directDrive(0,(newLeftTarget > 0 ? -1 * Math.abs(speed) : Math.abs(speed)), 0);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
            //robot.rightMotor2.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
    */
    public void encoderDriveX(double speed,
                              double xInches,
                              double timeoutS) {
        int newTarget;




        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newTarget = robot.centralMotor.getCurrentPosition() + (int)(xInches * COUNTS_PER_INCH);
            robot.centralMotor.setTargetPosition(newTarget);


            // Turn On RUN_TO_POSITION
            robot.centralMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




            // reset the timeout time and start motion.
            runtime.reset();
            robot.directDrive((newTarget > 0 ? -1 * Math.abs(speed) : Math.abs(speed)), 0, 0);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.centralMotor.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newTarget);
                telemetry.addData("Path2",  "Running at %7d",
                        robot.centralMotor.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;

            robot.centralMotor.setPower(0);
            //robot.rightMotor2.setPower(0);
            // Turn off RUN_TO_POSITION

            robot.centralMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    @Autonomous(name="BlueAutoMode", group="RobotHardwareClassThingmabobba")


    public static class BlueAutoMode extends ColinOpMode {
        RobotHardwareClassThingmabobba         robot   = new RobotHardwareClassThingmabobba();   // Use a Pushbot's hardware
        private ElapsedTime runtime = new ElapsedTime();
        BNO055IMU gyro;

        static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
        static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
        static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     DRIVE_SPEED             = 0.6;
        static final double     TURN_SPEED              = 0.5;
        static final double     FORWARD_SPEED           = 0.15;
        static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
        static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
        static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
        static final double     OPEN_SPEED              = 0.1;









        @Override
        public void runOpMode() {


           /*
            * Initialize the drive system variables.
            * The init() method of the hardware class does all the work here
            */
            robot.init(hardwareMap);
            gyro = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "Extendomatics3764.json";
            gyro.initialize(parameters);
            robot.leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            robot.rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors



            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Resetting Encoders");    //
            telemetry.update();


            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            idle();


            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0", "Starting at %7d :%7d",
                    robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();



            // Wait for the game to start (driver presses PLAY)
            waitForStart();



            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)

            /*encoderDriveY(DRIVE_SPEED, 17, 17, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDriveY(TURN_SPEED,   4.25, -4.25, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            encoderDriveY(DRIVE_SPEED, 4, 4, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            */
            //gyroTurn( TURN_SPEED, 45.0);   // Turn  CCW to -45 Degrees

            robot.centralMotor.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            robot.centralMotor.setPower(0);
            gyroDrive(DRIVE_SPEED, 28.0, 0.0);    // Drive FWD 48 inches

            robot.leftClaw.setPower(-FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            gyroDrive(DRIVE_SPEED, -3, 0.0);    // Drive FWD 48 inches
            sleep(10000);


            //encoderDriveX(DRIVE_SPEED, 10, 1.0);
            //gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second


            telemetry.addData("Path", "Complete");
            telemetry.update();

            sleep(3000);







        }
        public void gyroDrive ( double speed,
                                double distance,
                                double angle) {

            int     newLeftTarget;
            int     newRightTarget;
            int     moveCounts;
            double  max;
            double  error;
            double  steer;
            double  leftSpeed;
            double  rightSpeed;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                moveCounts = (int)(distance * COUNTS_PER_INCH);
                newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
                newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                robot.leftMotor.setTargetPosition(newLeftTarget);
                robot.rightMotor.setTargetPosition(newRightTarget);

                robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(Math.abs(speed), 0.0, 1.0);
                robot.leftMotor.setPower(speed);
                robot.rightMotor.setPower(speed);

                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive() &&
                        (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0)
                    {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    robot.leftMotor.setPower(leftSpeed);
                    robot.rightMotor.setPower(rightSpeed);

                    // Display drive status for the driver.
                    telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                    telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                    telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                            robot.rightMotor.getCurrentPosition());
                    telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                    telemetry.update();
                }

                // Stop all motion;
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        public void gyroTurn (  double speed, double angle) {

            // keep looping while we are still active, and not on heading.
            while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
                // Update telemetry & Allow time for other processes to run.
                telemetry.update();
            }
        }
        public void gyroHold( double speed, double angle, double holdTime) {

            ElapsedTime holdTimer = new ElapsedTime();

            // keep looping while we have time remaining.
            holdTimer.reset();
            while (opModeIsActive() && (holdTimer.time() < holdTime)) {
                // Update telemetry & Allow time for other processes to run.
                onHeading(speed, angle, P_TURN_COEFF);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }
        boolean onHeading(double speed, double angle, double PCoeff) {
            double   error ;
            double   steer ;
            boolean  onTarget = false ;
            double leftSpeed;
            double rightSpeed;

            // determine turn power based on +/- error
            error = getError(angle);

            if (Math.abs(error) <= HEADING_THRESHOLD) {
                steer = 0.0;
                leftSpeed  = 0.0;
                rightSpeed = 0.0;
                onTarget = true;
            }
            else {
                steer = getSteer(error, PCoeff);
                rightSpeed  = speed * steer;
                leftSpeed   = -rightSpeed;
            }

            // Send desired speeds to motors.
            robot.leftMotor.setPower(leftSpeed);
            robot.rightMotor.setPower(rightSpeed);

            // Display it for the driver.
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

            return onTarget;
        }
        public double getError(double targetAngle) {

            double robotError;

            // calculate error in -179 to +180 range  (
            robotError = targetAngle - gyro.getAngularOrientation().firstAngle;
            while (robotError > 180)  robotError -= 360;
            while (robotError <= -180) robotError += 360;
            return robotError;
        }
        public double getSteer(double error, double PCoeff) {
            return Range.clip(error * PCoeff, -1, 1);
        }
        /*
        *  Method to perfmorm a relative move, based on encoder counts.
        *  Encoders are not reset as the move is based on the current position.
        *  Move will stop if any of three conditions occur:
        *  1) Move gets to the desired position
        *  2) Move runs out of time
        *  3) Driver stops the opmode running.
        */
        /*public void encoderDriveY(double speed,
                                  double yInches,
                                  double timeoutS) {
            int newLeftTarget;
            int newRightTarget;




            // Ensure that the opmode is still active
            if (opModeIsActive()) {


                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(yInches * COUNTS_PER_INCH);
                newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(yInches * COUNTS_PER_INCH);
                robot.leftMotor.setTargetPosition(newLeftTarget);
                robot.rightMotor.setTargetPosition(newRightTarget);


                // Turn On RUN_TO_POSITION
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




                // reset the timeout time and start motion.
                runtime.reset();
                robot.directDrive(0,(newLeftTarget > 0 ? -1 * Math.abs(speed) : Math.abs(speed)), 0);


                // keep looping while we are still active, and there is time left, and both motors are running.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {


                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                    telemetry.addData("Path2",  "Running at %7d :%7d",
                            robot.leftMotor.getCurrentPosition(),
                            robot.rightMotor.getCurrentPosition());
                    telemetry.update();
                }


                // Stop all motion;
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                //robot.rightMotor2.setPower(0);
                // Turn off RUN_TO_POSITION
                robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //  sleep(250);   // optional pause after each move
            }
        }
        */
        public void encoderDriveX(double speed,
                                  double xInches,
                                  double timeoutS) {
            int newTarget;




            // Ensure that the opmode is still active
            if (opModeIsActive()) {


                // Determine new target position, and pass to motor controller
                newTarget = robot.centralMotor.getCurrentPosition() + (int)(xInches * COUNTS_PER_INCH);
                robot.centralMotor.setTargetPosition(newTarget);


                // Turn On RUN_TO_POSITION
                robot.centralMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




                // reset the timeout time and start motion.
                runtime.reset();
                robot.directDrive((newTarget > 0 ? -1 * Math.abs(speed) : Math.abs(speed)), 0, 0);


                // keep looping while we are still active, and there is time left, and both motors are running.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.centralMotor.isBusy())) {


                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d", newTarget);
                    telemetry.addData("Path2",  "Running at %7d",
                            robot.centralMotor.getCurrentPosition());
                    telemetry.update();
                }


                // Stop all motion;

                robot.centralMotor.setPower(0);
                //robot.rightMotor2.setPower(0);
                // Turn off RUN_TO_POSITION

                robot.centralMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //  sleep(250);   // optional pause after each move
            }
        }
    }
}
