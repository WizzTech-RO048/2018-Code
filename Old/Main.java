package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
@Autonomous(name="Gyro")
public class Main extends LinearOpMode {


    //VUFORIA
    VuforiaLocalizer vuforia;
    int col=-1;
    public int ceva(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AVR36zX/////AAAAmf4wKl7jz04GkSKCzFf6A/gIfIqXUXFiD8Q8smknNL1/7mQ7bfNDDX/GKq/vejsAFCypbnHQwmHunWd2pk/dOZY2Wi4Nj64UbWmDawkA3Jy9oiIfS4C7sfViotjeuhQZuuUuHOEDh63tJEX54rWgXUHYYpBUApz+2pB4ijjg+YipO05M9EEInFeUqL3rpEu1vuLq942L/tc7r2C/Am9W37dHlCMlIBYJ2f1RpTdi/6+WVFuiD5lhIyL6hGU9OMhmRzBrZeJWF0GNFHqi21JxiD9IpRIsk6KDqMdS/gYEWg20Lkk/vPEDbKXCrdm9iAQjlBAzjPq0gIE6i785JI8y61qzxohgI4PXEF0ciUqiC2UD";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if(vuMark==RelicRecoveryVuMark.LEFT) {
                telemetry.addData("left", vuMark);
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                col=0;
                return col;
            }
            if(vuMark==RelicRecoveryVuMark.CENTER) {
                telemetry.addData("center", vuMark);
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                col=1;
                return col;
            }
            if(vuMark==RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("right", vuMark);
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                col=2;
                return col;
            }
            telemetry.update();

        }
         return col;

    }
    //VUFORIA







    Initialization robot   = new Initialization();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro gyro    = null;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    //static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    //int VUMARK=ceva(); //VUFORIA CALL
    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.

        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }

        gyro.resetZAxisIntegrator();

        /**IFURILE
         *
         */
        gyroDrive(1,2,0);
        sleep(3000);
        gyroDrive(0.5,1,180);
        double  error;
        double  steer;
        error = getError(180);
        steer = getSteer(error, P_DRIVE_COEFF);
        telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
        telemetry.update();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        /*gyroDrive(DRIVE_SPEED, 2.0, 0.0);    // Drive FWD 48 inches
        gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
        gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
        gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches
        */
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget1;
        int     newLeftTarget2;
        int     newRightTarget1;
        int     newRightTarget2;
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

            newLeftTarget1 = robot.Roata_Stanga_Spate.getCurrentPosition() + moveCounts;
            newLeftTarget2 = robot.Roata_Dreapta_Spate.getCurrentPosition() + moveCounts;
            newRightTarget1 = robot.Roata_Dreapta_Spate.getCurrentPosition() + moveCounts;
            newRightTarget2 = robot.Roata_Dreapta_Spate.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.Roata_Stanga_Fata.setTargetPosition(newLeftTarget2);
            robot.Roata_Stanga_Spate.setTargetPosition(newLeftTarget1);
            robot.Roata_Dreapta_Fata.setTargetPosition(newRightTarget1);
            robot.Roata_Dreapta_Spate.setTargetPosition(newRightTarget2);

            robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.Roata_Stanga_Fata.setPower(speed);
            robot.Roata_Stanga_Spate.setPower(speed);
            robot.Roata_Dreapta_Fata.setPower(speed);
            robot.Roata_Dreapta_Spate.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while(robot.Roata_Stanga_Spate.isBusy()&&robot.Roata_Stanga_Fata.isBusy()&&
                    robot.Roata_Dreapta_Spate.isBusy()&&robot.Roata_Dreapta_Fata.isBusy()&&
                    opModeIsActive()) {

                // adjust relative speed based on heading error.
                error =0;// getError(angle);
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

                robot.Roata_Stanga_Fata.setPower(leftSpeed);
                robot.Roata_Stanga_Spate.setPower(leftSpeed);
                robot.Roata_Dreapta_Fata.setPower(rightSpeed);
                robot.Roata_Dreapta_Spate.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget1,  newRightTarget1, newLeftTarget2,  newRightTarget2);
                telemetry.addData("Actual",  "%7d:%7d",      robot.Roata_Stanga_Fata.getCurrentPosition(), robot.Roata_Stanga_Spate.getCurrentPosition(),
                        robot.Roata_Dreapta_Fata.getCurrentPosition(), robot.Roata_Dreapta_Spate.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.Roata_Stanga_Fata.setPower(0);
            robot.Roata_Stanga_Spate.setPower(0);
            robot.Roata_Dreapta_Fata.setPower(0);
            robot.Roata_Dreapta_Spate.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
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
        robot.Roata_Stanga_Fata.setPower(0);
        robot.Roata_Stanga_Spate.setPower(0);
        robot.Roata_Dreapta_Fata.setPower(0);
        robot.Roata_Dreapta_Spate.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
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
        robot.Roata_Stanga_Fata.setPower(leftSpeed);
        robot.Roata_Stanga_Spate.setPower(leftSpeed);
        robot.Roata_Dreapta_Fata.setPower(rightSpeed);
        robot.Roata_Dreapta_Spate.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        if(robotError<3 && robotError>-3)
            return 0;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public static class Initialization
    {
        /* Public OpMode members. */

        public DcMotor Roata_Stanga_Fata = null;
        public DcMotor Roata_Stanga_Spate = null;
        public DcMotor Roata_Dreapta_Fata = null;
        public DcMotor Roata_Dreapta_Spate = null;
        //public DcMotor Roata1 = null;
        //public DcMotor Roata2 = null;
        /* local OpMode members. */
        HardwareMap hwMap    =  null;
        private ElapsedTime period  = new ElapsedTime();

        public Initialization() {

        }


        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors
            Roata_Stanga_Fata = hwMap.get(DcMotor.class, "sf");
            Roata_Stanga_Spate = hwMap.get(DcMotor.class, "ss");
            Roata_Dreapta_Fata = hwMap.get(DcMotor.class, "df");
            Roata_Dreapta_Spate = hwMap.get(DcMotor.class, "ds");

            Roata_Stanga_Fata.setDirection(DcMotor.Direction.FORWARD);
            Roata_Stanga_Spate.setDirection(DcMotor.Direction.FORWARD);
            Roata_Dreapta_Fata.setDirection(DcMotor.Direction.REVERSE);
            Roata_Dreapta_Spate.setDirection(DcMotor.Direction.REVERSE);

            // Set all motors to zero power
            Roata_Stanga_Fata.setPower(0);
            Roata_Stanga_Spate.setPower(0);
            Roata_Dreapta_Fata.setPower(0);
            Roata_Dreapta_Spate.setPower(0);



        }

        public void Run(){
            Roata_Stanga_Fata.setPower(0.3);
            Roata_Stanga_Spate.setPower(0.3);
            Roata_Dreapta_Spate.setPower(0.3);
            Roata_Dreapta_Fata.setPower(0.3);

        }

        public void Stop(){
            Roata_Stanga_Fata.setPower(0);
            Roata_Stanga_Spate.setPower(0);
            Roata_Dreapta_Spate.setPower(0);
            Roata_Dreapta_Fata.setPower(0);
        }

        public void Stanga(){
            Roata_Stanga_Fata.setPower(-0.3);
            Roata_Stanga_Spate.setPower(-0.3);
            Roata_Dreapta_Spate.setPower(0.3);
            Roata_Dreapta_Fata.setPower(0.3);
        }

        public void Dreapta(){
            Roata_Stanga_Fata.setPower(0.3);
            Roata_Stanga_Spate.setPower(0.3);
            Roata_Dreapta_Spate.setPower(-0.3);
            Roata_Dreapta_Fata.setPower(-0.3);
        }
    }

}