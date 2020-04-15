package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "Rosu Departe")
public class Rosu_departe extends LinearOpMode {
    public static class Initialization {

        public DcMotor Roata_Stanga_Fata = null;
        public DcMotor Roata_Stanga_Spate = null;
        public DcMotor Roata_Dreapta_Fata = null;
        public DcMotor Roata_Dreapta_Spate = null;
        public DcMotor Brats = null;
        public DcMotor Bratd = null;
        HardwareMap hwMap = null;
        Servo brat;
        Servo bratsecund;
        Servo servo_stanga;
        Servo servo_dreapta;
        Servo prindere_stanga;
        Servo prindere_dreapta;
        ModernRoboticsI2cColorSensor colorSensor;

        public Initialization() {

        }


        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;
            bratsecund = hwMap.get(Servo.class, "bratsecund");
            brat = hwMap.get(Servo.class, "bratmingi");
            colorSensor = hwMap.get(ModernRoboticsI2cColorSensor.class, "jewels_sensor");
            servo_stanga = hwMap.get(Servo.class, "servostanga");
            servo_dreapta = hwMap.get(Servo.class, "servodreapta");
            prindere_stanga=hwMap.get(Servo.class,"prindere_stanga");
            prindere_dreapta=hwMap.get(Servo.class,"prindere_dreapta");
            Brats = hwMap.get(DcMotor.class, "brats");
            Bratd = hwMap.get(DcMotor.class, "bratd");
            Roata_Stanga_Fata = hwMap.get(DcMotor.class, "sf");
            Roata_Stanga_Spate = hwMap.get(DcMotor.class, "ss");
            Roata_Dreapta_Fata = hwMap.get(DcMotor.class, "df");
            Roata_Dreapta_Spate = hwMap.get(DcMotor.class, "ds");

            Roata_Stanga_Fata.setDirection(DcMotor.Direction.REVERSE);
            Roata_Stanga_Spate.setDirection(DcMotor.Direction.REVERSE);
            Roata_Dreapta_Fata.setDirection(DcMotor.Direction.FORWARD);
            Roata_Dreapta_Spate.setDirection(DcMotor.Direction.FORWARD);

            Bratd.setDirection(DcMotor.Direction.FORWARD);
            Brats.setDirection(DcMotor.Direction.FORWARD);
            // Set all motors to zero power
            Roata_Stanga_Fata.setPower(0);
            Roata_Stanga_Spate.setPower(0);
            Roata_Dreapta_Fata.setPower(0);
            Roata_Dreapta_Spate.setPower(0);
        }
    }

    Initialization robot = new Initialization();
    ModernRoboticsI2cGyro gyro = null;

    //static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double WHEEL_DIAMETER_INCHES = 10.16;
    static final double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        telemetry.addData(">", "Giroscopul se calibreaza");
        telemetry.update();

        gyro.calibrate();
        gyro.resetZAxisIntegrator();

        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Pregatit.");
        telemetry.update();

        robot.colorSensor.enableLed(true);
        robot.brat.setPosition(0.8);
        robot.bratsecund.setPosition(1);
        robot.servo_dreapta.setPosition(0.42);
        robot.servo_stanga.setPosition(0.58);
        robot.prindere_stanga.setPosition(0.3);
        robot.prindere_dreapta.setPosition(0.3);
        waitForStart();
        runtime.reset();

        int col=1;
        //int col = ceva();
        //int z = gyro.getIntegratedZValue();
        brat_mingi();
        Drive(-0.19,95);
        Rotire_dreapta(0.1,87);
        if (col == 0) {
            Drive(0.22, 4);
        } else if (col == 1) {
            Drive(0.22, 10);
        } else if (col == 2) {
            Drive(0.22, 18);
        }
        //Rotire_stanga(0.1, 87);
        Roti_verzi();
        //Plasare_cub();
        //Drive(-0.19, 10);
        //Drive(0.19, 10);
        Stop();

    }

    public void Corectare_beta(double speed, double z) {
        while (opModeIsActive() && gyro.getHeading() != z) {
            if (gyro.getHeading() > z)
                Dreapta(speed);
            if (gyro.getHeading() < z)
                Stanga(speed);
            if (gyro.getHeading() == z)
                stop();
        }
    }

    public void Run(double speed) {
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Stanga_Fata.setPower(speed);
        robot.Roata_Stanga_Spate.setPower(speed);
        robot.Roata_Dreapta_Spate.setPower(speed);
        robot.Roata_Dreapta_Fata.setPower(speed);
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Run_side(double speed) {
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Stanga_Fata.setPower(speed);
        robot.Roata_Stanga_Spate.setPower(-speed);
        robot.Roata_Dreapta_Spate.setPower(speed);
        robot.Roata_Dreapta_Fata.setPower(-speed);
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Drive(double speed,
                      double distance) {
        int stanga;
        int dreapta;
        int moveCounts;
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (opModeIsActive()) {
            distance = distance * 0.58;
            moveCounts = (int) (distance * COUNTS_PER_CM);
            //double directie;
            //directie = gyro.getIntegratedZValue();
            telemetry.addData("moveCounts= ", moveCounts);
            telemetry.update();
            stanga = robot.Roata_Stanga_Fata.getCurrentPosition() + moveCounts;
            dreapta = robot.Roata_Dreapta_Fata.getCurrentPosition() + moveCounts;
            if (speed > 0) {
                while (robot.Roata_Dreapta_Fata.getCurrentPosition() > -dreapta && robot.Roata_Dreapta_Spate.getCurrentPosition() > -dreapta
                        && robot.Roata_Stanga_Fata.getCurrentPosition() > -stanga && robot.Roata_Stanga_Spate.getCurrentPosition() > -stanga && opModeIsActive()) {
                    Run(speed);
                }
            } else if (speed < 0) {
                while (robot.Roata_Dreapta_Fata.getCurrentPosition() < dreapta && robot.Roata_Dreapta_Spate.getCurrentPosition() < dreapta
                        && robot.Roata_Stanga_Fata.getCurrentPosition() < stanga && robot.Roata_Stanga_Spate.getCurrentPosition() < stanga && opModeIsActive()) {
                    Run(speed);
                }
            }
            Stop();

        }
    }

    //+ in stanga
    public void Drive_side(double speed, double distance) {
        int fata;
        int spate;
        int moveCounts;
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (opModeIsActive()) {
            distance = distance * 0.58;
            moveCounts = (int) (distance * COUNTS_PER_CM);
            //double directie;
            //directie = gyro.getIntegratedZValue();
            telemetry.addData("moveCounts= ", moveCounts);
            telemetry.update();
            fata = robot.Roata_Stanga_Fata.getCurrentPosition() + moveCounts;
            spate = robot.Roata_Dreapta_Spate.getCurrentPosition() + moveCounts;
            if (speed > 0) {
                while (robot.Roata_Stanga_Fata.getCurrentPosition() > -fata && robot.Roata_Dreapta_Fata.getCurrentPosition() > -fata
                        && robot.Roata_Stanga_Spate.getCurrentPosition() > -spate && robot.Roata_Dreapta_Spate.getCurrentPosition() > -spate && opModeIsActive()) {
                    Run_side(speed);
                }
            } else if (speed < 0) {
                while (robot.Roata_Stanga_Fata.getCurrentPosition() > fata && robot.Roata_Dreapta_Fata.getCurrentPosition() > fata
                        && robot.Roata_Stanga_Spate.getCurrentPosition() > spate && robot.Roata_Dreapta_Spate.getCurrentPosition() > spate && opModeIsActive()) {
                    Run_side(speed);
                }
            }
            Stop();

        }
    }


    public void Stop() {
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Stanga_Fata.setPower(0);
        robot.Roata_Stanga_Spate.setPower(0);
        robot.Roata_Dreapta_Fata.setPower(0);
        robot.Roata_Dreapta_Spate.setPower(0);
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Dreapta(double speed) {
        if (opModeIsActive()) {
            robot.Roata_Stanga_Fata.setPower(speed);
            robot.Roata_Stanga_Spate.setPower(speed);
            robot.Roata_Dreapta_Spate.setPower(-speed);
            robot.Roata_Dreapta_Fata.setPower(-speed);
        }
    }

    public void Stanga(double speed) {
        if (opModeIsActive()) {
            robot.Roata_Stanga_Fata.setPower(-speed);
            robot.Roata_Stanga_Spate.setPower(-speed);
            robot.Roata_Dreapta_Spate.setPower(speed);
            robot.Roata_Dreapta_Fata.setPower(speed);
        }
        //stop();
    }

    //De pregatit dupa regionala
    /*
    public void Corectare(double directie) {

        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Stop();

        while(opModeIsActive() && gyro.getIntegratedZValue() < directie)
        {
            if(gyro.getIntegratedZValue() < directie)
            {
                Dreapta(0.2);
            }
            Dreapta(0.2);
            if(gyro.getIntegratedZValue() > directie)
            {
                Stanga(0.2);
            }
        }

        Stop();

        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    */
//SCHIMBATE INTRE ELE
    public void Rotire_stanga(double speed, int angle) {

        gyro.resetZAxisIntegrator();
        int currentHeading = gyro.getIntegratedZValue();
        int target = currentHeading + angle - 3;
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (currentHeading < target && opModeIsActive()) {
            telemetry.addData(">", "Orientarea robotului = %d grade", gyro.getIntegratedZValue());
            telemetry.update();
            Stanga(speed);
            currentHeading = gyro.getIntegratedZValue();
        }
        Stop();
    }

    public void Rotire_dreapta(double speed, int angle) {
        angle = -angle;
        gyro.resetZAxisIntegrator();
        int currentHeading = gyro.getIntegratedZValue();
        int target = currentHeading + angle + 3;
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (currentHeading > target && opModeIsActive()) {
            telemetry.addData(">", "Orientarea robotului = %d grade", gyro.getIntegratedZValue());
            telemetry.update();
            Dreapta(speed);
            currentHeading = gyro.getIntegratedZValue();
        }
        Stop();
    }


    VuforiaLocalizer vuforia;

    public int ceva() {
        int col = -1;
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

        while (opModeIsActive() && col == -1) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addData("left", vuMark);
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                col = 0;
                return col;
            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                telemetry.addData("center", vuMark);
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                col = 1;
                return col;
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addData("right", vuMark);
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                col = 2;
                return col;
            }
            telemetry.update();

        }
        return col;

    }


    //Pentru Rosu
    public void brat_mingi() throws InterruptedException {
        boolean ok = false;
        int okbrat = 0;
        int okbrats = 1;
        double power = 0.5;

        robot.brat.setPosition(0);
        sleep(1000);
        robot.bratsecund.setPosition(0);
        sleep(4000);
        int v = 1;
        while (opModeIsActive() && (v != 0)) {
            telemetry.addData("Color Name", robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.update();
            if (robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 10 && ok == false) {
                ok = true;
                Rotire_dreapta(0.1, 10);
                TimeUnit.MILLISECONDS.sleep(250);
                robot.bratsecund.setPosition(1);
                TimeUnit.MILLISECONDS.sleep(500);
                robot.brat.setPosition(0.7);
                Rotire_stanga(0.1, 10);
                sleep(250);
                v = 0;

            }
            if (robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 3 && ok == false) {
                ok = true;
                Rotire_stanga(0.1, 10);
                TimeUnit.MILLISECONDS.sleep(250);
                robot.bratsecund.setPosition(1);
                TimeUnit.MILLISECONDS.sleep(500);
                robot.brat.setPosition(0.7);
                Rotire_dreapta(0.1, 10);
                sleep(250);
                v = 0;
            } else {
                robot.bratsecund.setPosition(1);
                sleep(500);
                robot.brat.setPosition(0.7);
                v = 0;
            }
        }
    }


    public void Roti_verzi() throws InterruptedException {
        robot.Brats.setPower(-1);
        robot.Bratd.setPower(1);
        TimeUnit.MILLISECONDS.sleep(3000);
        robot.Bratd.setPower(0);
        robot.Brats.setPower(0);

    }

    public void Plasare_cub() {
        sleep(1500);
        robot.servo_stanga.setPosition(0.15);
        robot.servo_dreapta.setPosition(0.85);
        sleep(1000);
        robot.servo_dreapta.setPosition(0.42);
        robot.servo_stanga.setPosition(0.58);
    }
}