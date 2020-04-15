
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
        import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
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


@Autonomous (name = "Autonom_Vlad_18_03_18")
public class Autonom_Vlad_18_03_18 extends LinearOpMode
{

    private  ElapsedTime runtime = new ElapsedTime();
    public DcMotor Roata_Stanga_Fata = null;
    public DcMotor Roata_Stanga_Spate = null;
    public DcMotor Roata_Dreapta_Fata = null;
    public DcMotor Roata_Dreapta_Spate = null;
    public DcMotor Bratd=null;
    public DcMotor Brats=null;
    //HardwareMap hwMap = null;
    Servo bratmingi;
    Servo bratsecund;
    Servo servo_stanga;
    Servo servo_dreapta;
    ModernRoboticsI2cColorSensor colorSensor;
    ModernRoboticsI2cGyro gyro = null;

    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double WHEEL_DIAMETER_INCHES = 10.16;
    static final double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    @Override
    public void runOpMode() throws InterruptedException
    {

        servo_stanga=hardwareMap.get(Servo.class,"servo_stanga");
        servo_dreapta=hardwareMap.get(Servo.class,"servo_dreapta");
        bratsecund = hardwareMap.get(Servo.class, "bratsecund");
        bratmingi = hardwareMap.get(Servo.class, "bratmingi");
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "jewls_sensor");
        Roata_Stanga_Fata = hardwareMap.get(DcMotor.class, "sf");
        Roata_Stanga_Spate = hardwareMap.get(DcMotor.class, "ss");
        Roata_Dreapta_Fata = hardwareMap.get(DcMotor.class, "df");
        Roata_Dreapta_Spate = hardwareMap.get(DcMotor.class, "ds");
        Brats = hardwareMap.get(DcMotor.class, "brats");
        Bratd = hardwareMap.get(DcMotor.class, "bratd");
        Bratd.setDirection(DcMotor.Direction.FORWARD);
        Brats.setDirection(DcMotor.Direction.REVERSE);
        Roata_Stanga_Fata.setDirection(DcMotor.Direction.REVERSE);
        Roata_Stanga_Spate.setDirection(DcMotor.Direction.REVERSE);
        Roata_Dreapta_Fata.setDirection(DcMotor.Direction.FORWARD);
        Roata_Dreapta_Spate.setDirection(DcMotor.Direction.FORWARD);
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

        colorSensor.enableLed(true);
        Roata_Stanga_Fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Roata_Stanga_Spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Roata_Dreapta_Spate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Roata_Dreapta_Fata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //int col=ceva();
        int ok=0;
        int col=0;
        int z=gyro.getIntegratedZValue();
        servo_dreapta.setPosition(0.42);
        servo_stanga.setPosition(0.58);
        brat_mingi();
       /*if(col==0)
        {
            Drive(0.175,90);
            ok=1;
        }
        else if(col==1)
        {
            Drive(0.175,110);
            ok=1;
        }
        else if(col==2)
        {
            Drive(0.175,130);
            ok=1;
        }
        */
        Drive(0.4,90);
        //ok=1;
        //Corectare_beta(0.3,z);
        Rotire_dreapta(0.2,90);
        // Stop();
        //if(ok==1) {
        sleep(500);
        Roti_verzi();
        sleep(1000);
        Plasare_cub();
        sleep(1000);
        //}
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
        Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Stanga_Fata.setPower(speed);
        Roata_Stanga_Spate.setPower(speed);
        Roata_Dreapta_Spate.setPower(speed);
        Roata_Dreapta_Fata.setPower(speed);
        Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Drive(double speed,
                      double distance)
    {
        int stanga;
        int dreapta;
        int moveCounts;

        Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(opModeIsActive())
        {
            distance=distance*0.58;
            moveCounts =(int)  (distance * COUNTS_PER_CM);
            //double directie;
            //directie = gyro.getIntegratedZValue();
            telemetry.addData("moveCounts= ", moveCounts);
            telemetry.update();
            stanga = Roata_Stanga_Spate.getCurrentPosition() + moveCounts;
            dreapta = Roata_Dreapta_Spate.getCurrentPosition() + moveCounts;
            if(speed>0) {
                while (Roata_Dreapta_Fata.getCurrentPosition() > -dreapta && Roata_Dreapta_Spate.getCurrentPosition() > -dreapta
                        && Roata_Stanga_Fata.getCurrentPosition() > -stanga && Roata_Dreapta_Spate.getCurrentPosition() > -stanga && opModeIsActive()) {
                    Run(speed);
                }
            }
            else if(speed<0) {
                while (Roata_Dreapta_Fata.getCurrentPosition() < dreapta && Roata_Dreapta_Spate.getCurrentPosition() < dreapta
                        && Roata_Stanga_Fata.getCurrentPosition() < stanga && Roata_Dreapta_Spate.getCurrentPosition() < stanga && opModeIsActive()) {
                    Run(speed);
                }
            }
            Stop();

        }
    }


    public void Stop()
    {
        Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Stanga_Fata.setPower(0);
        Roata_Stanga_Spate.setPower(0);
        Roata_Dreapta_Fata.setPower(0);
        Roata_Dreapta_Spate.setPower(0);
        Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Dreapta(double speed)
    {
        if (opModeIsActive())
        {
            Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Roata_Stanga_Fata.setPower(-speed);
            Roata_Stanga_Spate.setPower(-speed);
            Roata_Dreapta_Spate.setPower(speed);
            Roata_Dreapta_Fata.setPower(speed);
            Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void Stanga(double speed)
    {
        if(opModeIsActive()) {
            Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Roata_Stanga_Fata.setPower(speed);
            Roata_Stanga_Spate.setPower(speed);
            Roata_Dreapta_Spate.setPower(-speed);
            Roata_Dreapta_Fata.setPower(-speed);
            Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //stop();
    }

    //De pregatit dupa regionala
    /*
    public void Corectare(double directie) {

        robot.leftdrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        robot.leftdrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (currentHeading < target && opModeIsActive()) {
            telemetry.addData(">", "Orientarea robotului = %d grade", gyro.getIntegratedZValue());
            telemetry.update();
            Dreapta(speed);
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
        Roata_Stanga_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Stanga_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Dreapta_Fata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Roata_Dreapta_Spate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (currentHeading > target && opModeIsActive()) {
            telemetry.addData(">", "Orientarea robotului = %d grade", gyro.getIntegratedZValue());
            telemetry.update();
            Stanga(speed);
            currentHeading = gyro.getIntegratedZValue();
        }
        Stop();
    }


    VuforiaLocalizer vuforia;
    public int ceva()
    {
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
    public void brat_mingi()
    {
        boolean ok=false;
        int okbrat=0;
        int okbrats=1;
        double power= 0.5;

        bratmingi.setPosition(0);
        sleep(1000);
        bratsecund.setPosition(0);
        int v=1;
        while(opModeIsActive() && (v != 0) )
        {
            telemetry.addData("Color Name", colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.update();


            if (colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 3 && ok == false) {
                ok = true;
                Rotire_dreapta(0.175,10);
                sleep(250);
                bratsecund.setPosition(1);
                sleep(500);
                bratmingi.setPosition(0.65);
                Rotire_stanga(0.175,10);
                sleep(250);
                v=0;

            }
            if (colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 10 && ok == false) {
                ok = true;
                Rotire_stanga(0.175,10);
                sleep(250);
                bratsecund.setPosition(1);
                sleep(500);
                bratmingi.setPosition(0.65);
                Rotire_dreapta(0.175,10);
                sleep(250);
                v=0;
            }
        }
    }

    public void Roti_verzi()
    {
        Bratd.setPower(0.75);
        Brats.setPower(0.75);
        sleep(1500);
        Bratd.setPower(0);
        Brats.setPower(0);

    }

    public void Plasare_cub()
    {
        servo_stanga.setPosition(0.15);
        servo_dreapta.setPosition(0.85);
        sleep(1000);
    }

}