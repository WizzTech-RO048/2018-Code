package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
import java.util.concurrent.TimeUnit;


@Autonomous (name = "Autonom_v1_0")
public class Autonom_v1_0 extends LinearOpMode {

    public static class Initialization {

        public DcMotor Roata_Stanga_Fata = null;
        public DcMotor Roata_Stanga_Spate = null;
        public DcMotor Roata_Dreapta_Fata = null;
        public DcMotor Roata_Dreapta_Spate = null;
        HardwareMap hwMap = null;
        CRServo brat;
        CRServo bratroti;
        Servo bratsecund;
        Servo servo_stanga;
        Servo servo_dreapta;
        ModernRoboticsI2cColorSensor colorSensor;

        public Initialization() {

        }


        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;
            bratroti=hwMap.get(CRServo.class,"bratroti");
            bratsecund = hwMap.get(Servo.class, "bratsecund");
            brat = hwMap.get(CRServo.class, "bratmingi");
            colorSensor = hwMap.get(ModernRoboticsI2cColorSensor.class, "jewls_sensor");
            servo_stanga = hwMap.get(Servo.class, "servo_stanga");
            servo_dreapta = hwMap.get(   Servo.class, "servo_dreapta");


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
    static final double WHEEL_DIAMETER_INCHES = 10.16;
    static final double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        telemetry.addData(">", "Giroscopul se calibreaza");
        telemetry.update();

        gyro.calibrate();
        gyro.resetZAxisIntegrator();

        while(!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Pregatit.");
        telemetry.update();

        waitForStart();
        runtime.reset();
        robot.colorSensor.enableLed(true);
        robot.brat.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        brat_roti();
        pune_cub();
        int col=ceva();
        brat_mingi();
        pune_cub();

        //int col=ceva();
        int z=gyro.getIntegratedZValue();


        if(col==0)
        {
            Drive(0.175,90);
        }
        else if(col==1)
        {
            Drive(0.175,110);
        }
        else if(col==2)
        {
            Drive(0.175,130);
        }
        Corectare_beta(0.1,z);
        Rotire_dreapta(0.1,90);
        pune_cub();
        Stop();
    }

    public void Corectare_beta(double speed, double z) {
        while(opModeIsActive() && gyro.getHeading()!=z) {
            if(gyro.getHeading()<z)
                Dreapta(speed);
            if(gyro.getHeading()>z)
                Stanga(speed);
            if(gyro.getHeading()==z)
                stop();
        }
    }

    public void Run(double speed)
    {
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

    public void Drive(double speed,
                      double distance)
    {
        int stanga;
        int dreapta;
        int moveCounts;

        robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(opModeIsActive())
        {
            distance=distance*0.58;
            moveCounts =(int)  (distance * COUNTS_PER_CM);
            //double directie;
            //directie = gyro.getIntegratedZValue();
            telemetry.addData("moveCounts= ", moveCounts);
            telemetry.update();
            stanga = robot.Roata_Stanga_Spate.getCurrentPosition() + moveCounts;
            dreapta = robot.Roata_Dreapta_Spate.getCurrentPosition() + moveCounts;
            if(speed>0) {
                while (robot.Roata_Dreapta_Fata.getCurrentPosition() > -dreapta && robot.Roata_Dreapta_Spate.getCurrentPosition() > -dreapta
                        && robot.Roata_Stanga_Fata.getCurrentPosition() > -stanga && robot.Roata_Dreapta_Spate.getCurrentPosition() > -stanga && opModeIsActive()) {
                    Run(speed);
                }
            }
            else if(speed<0) {
                while (robot.Roata_Dreapta_Fata.getCurrentPosition() < dreapta && robot.Roata_Dreapta_Spate.getCurrentPosition() < dreapta
                        && robot.Roata_Stanga_Fata.getCurrentPosition() < stanga && robot.Roata_Dreapta_Spate.getCurrentPosition() < stanga && opModeIsActive()) {
                    Run(speed);
                }
            }
            Stop();

        }
    }


    public void Stop()
    {
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

    public void Dreapta(double speed)
    {
        if (opModeIsActive())
        {
            robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Roata_Stanga_Fata.setPower(-speed);
            robot.Roata_Stanga_Spate.setPower(-speed);
            robot.Roata_Dreapta_Spate.setPower(speed);
            robot.Roata_Dreapta_Fata.setPower(speed);
            robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void Stanga(double speed)
    {
        if(opModeIsActive()) {
            robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Roata_Stanga_Fata.setPower(speed);
            robot.Roata_Stanga_Spate.setPower(speed);
            robot.Roata_Dreapta_Spate.setPower(-speed);
            robot.Roata_Dreapta_Fata.setPower(-speed);
            robot.Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void Rotire_stanga(double speed, int angle)
    {

        gyro.resetZAxisIntegrator();
        int currentHeading =  gyro.getIntegratedZValue();
        int target = currentHeading + angle;
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

    public void Rotire_dreapta(double speed, int angle)
    {
        angle=-angle;
        gyro.resetZAxisIntegrator();
        int currentHeading =  gyro.getIntegratedZValue();
        int target = currentHeading + angle;
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

        while (opModeIsActive() && col==-1) {

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

    //PENTRU ECHIPA Albastru
    public void brat_mingi() throws InterruptedException {
        boolean ok=false;
        int okbrat=0;
        int okbrats=1;
        double power= 0.5;

        if (okbrat == 0) {
            robot.brat.setPower(0.7);
            TimeUnit.MILLISECONDS.sleep(400);
            robot.brat.setPower(0.0);
            okbrat=1;
        }
        if(okbrat==1)
            robot.bratsecund.setPosition(0);
        int v=1;
        while(opModeIsActive() && v!=0)
        {
            telemetry.addData("Color Name", robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.update();


            if (robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 3 && ok == false) {
                ok = true;
                Rotire_dreapta(0.175,10);
                sleep(250);

                if(okbrat==1 && okbrats==1) {
                    robot.bratsecund.setPosition(1.8);
                    okbrats=0;
                }
                robot.brat.setDirection(DcMotorSimple.Direction.FORWARD);
                if(okbrat==1) {
                    robot.brat.setPower(1);
                    TimeUnit.MILLISECONDS.sleep(1500);
                    robot.brat.setPower(0);
                    okbrat = 0;
                }
                Rotire_stanga(0.175,10);
                sleep(250);
                v=0;

            }
            if (robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 10 && ok == false) {
                ok = true;
                Rotire_stanga(0.175,10);
                sleep(250);

                if(okbrat==1 && okbrats==1) {
                    robot.bratsecund.setPosition(1.8);
                    okbrats=0;
                }
                robot.brat.setDirection(DcMotorSimple.Direction.FORWARD);
                if(okbrat==1) {
                    robot.brat.setPower(1);
                    TimeUnit.MILLISECONDS.sleep(1500);
                    robot.brat.setPower(0);
                    okbrat = 0;
                }
                Rotire_dreapta(0.175,10);
                sleep(250);
                v=0;
            }
        }
    }

    public void brat_roti() throws InterruptedException {
        //pentru teste
         robot.bratroti.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.bratroti.setPower(1);
        //ALA BUN
/*
        robot.bratroti.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.bratroti.setPower(1);
        TimeUnit.SECONDS.sleep(5);
        robot.bratroti.setPower(0);
*/
    }


    public void pune_cub(){
        double  pozs = 0;
        double pozd=0;

           /* if (pozs == pozclasics && pozd == pozclasicd)
            {
                for (int i = 1; i <= 12; i = i + 2) {
                    pozs = pozs - 2;
                    pozd = pozd + 2;
                    servo_stanga.setPosition(pozs);
                    servo_dreapta.setPosition(pozd);
                    sleep(10);
                }
             }
             */

        pozd=1.8;
        pozs=0;
        robot.servo_dreapta.setPosition(pozd);
        robot.servo_stanga.setPosition(pozs);
        pozs=0;
        pozd=0;


    }
}