package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name = "The Real G")
public class autonom_gyro extends LinearOpMode {

    public static class Initialization {
        /* Public OpMode members. */

        public DcMotor Roata_Stanga_Fata = null;
        public DcMotor Roata_Stanga_Spate = null;
        public DcMotor Roata_Dreapta_Fata = null;
        public DcMotor Roata_Dreapta_Spate = null;
        HardwareMap hwMap = null;
        private ElapsedTime period = new ElapsedTime();


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
            //Roata1 = hwMap.get(DcMotor.class, "brat1");
            //Roata2 = hwMap.get(DcMotor.class, "brat2");

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



        public void Rotire(double x) {
            Roata_Stanga_Fata.setPower(-x);
            Roata_Stanga_Spate.setPower(-x);
            Roata_Dreapta_Spate.setPower(x);
            Roata_Dreapta_Fata.setPower(x);
        }

        public void Stanga() {
            Roata_Stanga_Fata.setPower(-0.3);
            Roata_Stanga_Spate.setPower(-0.3);
            Roata_Dreapta_Spate.setPower(0.3);
            Roata_Dreapta_Fata.setPower(0.3);
        }

        public void Dreapta() {
            Roata_Stanga_Fata.setPower(0.3);
            Roata_Stanga_Spate.setPower(0.3);
            Roata_Dreapta_Spate.setPower(-0.3);
            Roata_Dreapta_Fata.setPower(-0.3);
        }
    }

    Initialization robot = new Initialization();
    ModernRoboticsI2cGyro gyro = null;

    static final double COUNTS_PER_MOTOR_REV = 1440;
    //static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    @Override
    public void runOpMode() {


        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        gyro.calibrate();

        telemetry.addData("gyro", "X:" + gyro.rawX()/100 + " Y:" + gyro.rawY()/100 + " Z:" + gyro.rawZ()/100);

        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

        waitForStart();
        robot.Rotire(0.5);
        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        gyro.calibrate();


        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();


        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
        }

        gyro.resetZAxisIntegrator();


        Drive(0.5, 5);
        robot.Roata_Stanga_Fata.setPower(0);
        robot.Roata_Stanga_Spate.setPower(0);
        robot.Roata_Dreapta_Fata.setPower(0);
        robot.Roata_Dreapta_Spate.setPower(0);


    }
    public void Drive(double speed,
                      double distance)
    {

        int newLeftTarget1;
        int newLeftTarget2;
        int newRightTarget1;
        int newRightTarget2;
        int moveCounts;

        // Ensure that the opmode is still active!!

        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moveCounts = (int) (distance * COUNTS_PER_INCH);

        newLeftTarget1 = robot.Roata_Stanga_Fata.getCurrentPosition() + moveCounts;
        newLeftTarget2 = robot.Roata_Stanga_Spate.getCurrentPosition() + moveCounts;
        newRightTarget1 =robot.Roata_Dreapta_Fata.getCurrentPosition() + moveCounts;
        newRightTarget2 =robot.Roata_Dreapta_Spate.getCurrentPosition() + moveCounts;


        robot.Roata_Stanga_Fata.setTargetPosition(newLeftTarget1);
        robot.Roata_Stanga_Spate.setTargetPosition(newLeftTarget2);
        robot.Roata_Dreapta_Fata.setTargetPosition(newRightTarget1);
        robot.Roata_Dreapta_Spate.setTargetPosition(newRightTarget2);

        robot.Roata_Stanga_Fata.setPower(speed);
        robot.Roata_Stanga_Spate.setPower(speed);
        robot.Roata_Dreapta_Spate.setPower(speed);
        robot.Roata_Dreapta_Fata.setPower(speed);

        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
}