package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



@Autonomous (name = "pls workeyx")
public class net extends LinearOpMode {

    public static class Initialization {

        public DcMotor Roata_Stanga_Fata = null;
        public DcMotor Roata_Stanga_Spate = null;
        public DcMotor Roata_Dreapta_Fata = null;
        public DcMotor Roata_Dreapta_Spate = null;
        HardwareMap hwMap = null;


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


    }

    Initialization robot = new Initialization();
    ModernRoboticsI2cGyro gyro = null;

    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    @Override
    public void runOpMode() {

       /* int Cod;
        Cod = ceva();*/

        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        telemetry.addData(">", "Giroscopul se calibreaza");
        telemetry.update();

        gyro.calibrate();
        gyro.resetZAxisIntegrator();

        telemetry.addData(">", "Robot Pregatit.");
        telemetry.update();

        waitForStart();
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!isStarted())
        {
            telemetry.addData(">", "Orientarea robotului = %d grade", gyro.getHeading());
            telemetry.update();
        }
        Drive(0.175, 5);
        sleep(2000);
        Drive_side(0.175,5);
        //sleep(1000);
        //Rotire(0.1, 180);
        //Rotire(0.2,90);
        stop();

    }


    public void Drive(double speed, double distance) {

        int stanga;
        int dreapta;
        int moveCounts;

        if (opModeIsActive()) {
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            double directie;
            directie = gyro.getHeading();
            telemetry.addData("moveCounts= ", moveCounts);
            telemetry.update();
            stanga = robot.Roata_Stanga_Spate.getCurrentPosition() + moveCounts;
            dreapta = robot.Roata_Dreapta_Spate.getCurrentPosition() + moveCounts;

            /*micsorare viteza cand suntem aproape
              if (Math.abs(target - rightFlipMotor.getTargetPosition()) < 100)
                 speed = 0.1;
            else
                speed = 0.8;
             */
            /*robot.Roata_Stanga_Fata.setTargetPosition(stanga);
            robot.Roata_Stanga_Spate.setTargetPosition(stanga);
            robot.Roata_Dreapta_Fata.setTargetPosition(dreapta);
            robot.Roata_Dreapta_Spate.setTargetPosition(dreapta);

            robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

            while (robot.Roata_Dreapta_Fata.getCurrentPosition() < dreapta && robot.Roata_Dreapta_Spate.getCurrentPosition() < dreapta
                    && robot.Roata_Stanga_Fata.getCurrentPosition() < stanga && robot.Roata_Dreapta_Spate.getCurrentPosition() < stanga && opModeIsActive()) {
                robot.Roata_Stanga_Fata.setPower(-speed);
                robot.Roata_Stanga_Spate.setPower(-speed);
                robot.Roata_Dreapta_Spate.setPower(-speed);
                robot.Roata_Dreapta_Fata.setPower(-speed);

            }

           /* while(robot.Roata_Dreapta_Fata.isBusy() && robot.Roata_Dreapta_Spate.isBusy() &&
                    robot.Roata_Stanga_Spate.isBusy() && robot.Roata_Stanga_Fata.isBusy() && opModeIsActive()) {
            */

            Stop();

            robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }


    public void Drive_side(double speed,double distance) {

        int spate,fata;
        int moveCounts;

        if (opModeIsActive()) {
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            double directie;
            directie = gyro.getHeading();
            telemetry.addData("moveCounts= ", moveCounts);
            telemetry.update();
            spate = robot.Roata_Stanga_Spate.getCurrentPosition() + moveCounts;
            fata = robot.Roata_Stanga_Fata.getCurrentPosition() + moveCounts;

            /*micsorare viteza cand suntem aproape
              if (Math.abs(target - rightFlipMotor.getTargetPosition()) < 100)
                 speed = 0.1;
            else
                speed = 0.8;
             */
            /*robot.Roata_Stanga_Fata.setTargetPosition(stanga);
            robot.Roata_Stanga_Spate.setTargetPosition(stanga);
            robot.Roata_Dreapta_Fata.setTargetPosition(dreapta);
            robot.Roata_Dreapta_Spate.setTargetPosition(dreapta);

            robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

            while (robot.Roata_Dreapta_Fata.getCurrentPosition() < fata && robot.Roata_Dreapta_Spate.getCurrentPosition() < fata
                    && robot.Roata_Stanga_Fata.getCurrentPosition() < spate && robot.Roata_Dreapta_Spate.getCurrentPosition() < spate && opModeIsActive()) {
                robot.Roata_Stanga_Fata.setPower(speed);
                robot.Roata_Stanga_Spate.setPower(speed);
                robot.Roata_Dreapta_Spate.setPower(-speed);
                robot.Roata_Dreapta_Fata.setPower(-speed);

            }

           /* while(robot.Roata_Dreapta_Fata.isBusy() && robot.Roata_Dreapta_Spate.isBusy() &&
                    robot.Roata_Stanga_Spate.isBusy() && robot.Roata_Stanga_Fata.isBusy() && opModeIsActive()) {
            */

            Stop();

            robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }
    public void Stop()
    {

        robot.Roata_Stanga_Fata.setPower(0);
        robot.Roata_Stanga_Spate.setPower(0);
        robot.Roata_Dreapta_Fata.setPower(0);
        robot.Roata_Dreapta_Spate.setPower(0);
    }

    public void Stanga(double speed)
    {
        if (opModeIsActive())
        {
            robot.Roata_Stanga_Fata.setPower(-speed);
            robot.Roata_Stanga_Spate.setPower(-speed);
            robot.Roata_Dreapta_Spate.setPower(speed);
            robot.Roata_Dreapta_Fata.setPower(speed);
        }
    }

    public void Dreapta(double speed)
    {
        if(opModeIsActive()) {
            robot.Roata_Stanga_Fata.setPower(speed);
            robot.Roata_Stanga_Spate.setPower(speed);
            robot.Roata_Dreapta_Spate.setPower(-speed);
            robot.Roata_Dreapta_Fata.setPower(-speed);
        }
    }

    public void Corectare(double directie) {

        if (opModeIsActive() && gyro.getIntegratedZValue()!=directie)
        {
            if(gyro.getIntegratedZValue() < directie)
            {
                Dreapta(0.200);
            }
            if(gyro.getIntegratedZValue() > directie)
            {
                Stanga(0.200);
            }
        }

    }


    public void Rotire(double speed, double angle)
    {
        int goalReached = 0;
        double goalAngle,speed1;
        if(angle>0)
            speed1=speed;
        else if(angle==0)
            speed1=0;
        else
            speed1=speed*(-1);
        goalAngle=gyro.getIntegratedZValue()+angle;
        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (goalReached==0 && opModeIsActive()) {
            if(gyro.getIntegratedZValue()==goalAngle || gyro.getIntegratedZValue()+1==goalAngle || gyro.getIntegratedZValue()-1==goalAngle) {
                goalReached = 1;
                stop();
            }
            else {
                Dreapta(speed1);
                telemetry.addData(">", "Orientarea robotului = %d grade", gyro.getIntegratedZValue());
                telemetry.addData(">", "%.3f", goalAngle);
                telemetry.update();
            }
        }
    }

    VuforiaLocalizer vuforia;

    public int ceva(){
        int col=-1;
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

}