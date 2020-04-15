package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;


@TeleOp(name="Octopussy")
public class octopussy   extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Roata_Stanga_Fata = null;
    private DcMotor Roata_Stanga_Spate = null;
    private DcMotor Roata_Dreapta_Fata = null;
    private DcMotor Roata_Dreapta_Spate = null;
    private DcMotor Brats = null;
    private DcMotor Bratd = null;
    private DcMotor motorplatforma = null;
    //static final int    CYCLE_MS    =   50;     // period of each cycle
    CRServo brat;
    Servo bratsecund;
    Servo servo_stanga;
    Servo servo_dreapta;
    CRServo bratroti;
    double  pozs = 0;
    double pozd=0;
    double pozclasics=0.68;
    double pozclasicd=0.3;
    public void PlatformaCuburi() {

        //de pus mai jos
        if (gamepad1.x) {
            motorplatforma.setPower(1);
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
            pozs=0.56;
            pozd=0.42;
            servo_dreapta.setPosition(pozd);
            servo_stanga.setPosition(pozs);
        }
        else if(gamepad1.y)
        {
            motorplatforma.setPower(-1);
        }
        else {
            motorplatforma.setPower(0);
        }
       /* if (gamepad1.x && MotorPlatforma1.getCurrentPosition() > 0 && MotorPlatforma1.getCurrentPosition() < 480
                && MotorPlatforma2.getCurrentPosition() > 0  && MotorPlatforma2.getCurrentPosition() < 480)
        {
            MotorPlatforma1.setPower(0.7);
            MotorPlatforma2.setPower(-0.7);
        }
        else if(gamepad1.y && MotorPlatforma1.getCurrentPosition() > 0 && MotorPlatforma1.getCurrentPosition() < 480
                && MotorPlatforma2.getCurrentPosition() > 0  && MotorPlatforma2.getCurrentPosition() < 480)
        {
            MotorPlatforma1.setPower(-0.7);
            MotorPlatforma2.setPower(0.7);
        }
        else
        {
            MotorPlatforma1.setPower(0.0);
            MotorPlatforma2.setPower(0.0);
        }
        */
    }
    public void Servouri() {
        /*
        if(gamepad1.a)
            clawposition += armspeed;
        else
        if(gamepad1.b)
            clawposition -=armspeed;
         */
        //De pus mai jos
        if (gamepad1.a) {

            pozd=1.8;
            pozs=0;
            servo_dreapta.setPosition(pozd);
            servo_stanga.setPosition(pozs);
        }
        if (gamepad1.b) {
            pozd=0.32;
            pozs=0.66;
            servo_dreapta.setPosition(pozd);
            servo_stanga.setPosition(pozs);
        }
    }
    int a=0,b=0;
    public void Init()
    {
        bratsecund = hardwareMap.get(Servo.class, "bratsecund");
        brat = hardwareMap.get(CRServo.class, "bratmingi");
        bratroti=hardwareMap.get(CRServo.class,"bratroti");
        bratsecund.resetDeviceConfigurationForOpMode();
        brat.resetDeviceConfigurationForOpMode();
        bratroti.resetDeviceConfigurationForOpMode();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Roata_Stanga_Fata = hardwareMap.get(DcMotor.class, "sf");
        Roata_Stanga_Spate = hardwareMap.get(DcMotor.class, "ss");
        Roata_Dreapta_Fata = hardwareMap.get(DcMotor.class, "df");
        Roata_Dreapta_Spate = hardwareMap.get(DcMotor.class, "ds");
        Brats = hardwareMap.get(DcMotor.class, "brats");
        Bratd = hardwareMap.get(DcMotor.class, "bratd");


        motorplatforma = hardwareMap.get(DcMotor.class, "platforma");
        //MotorPlatforma2 = hardwareMap.get(DcMotor.class, "platforma2");
        servo_stanga = hardwareMap.get(Servo.class, "servo_stanga");
        servo_dreapta = hardwareMap.get(   Servo.class, "servo_dreapta");


        Roata_Stanga_Fata.setDirection(DcMotor.Direction.REVERSE);
        Roata_Stanga_Spate.setDirection(DcMotor.Direction.REVERSE);
        Roata_Dreapta_Fata.setDirection(DcMotor.Direction.FORWARD);
        Roata_Dreapta_Spate.setDirection(DcMotor.Direction.FORWARD);
        Bratd.setDirection(DcMotor.Direction.FORWARD);
        Brats.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        //servo_stanga.setPosition(1.8);
        //servo_dreapta.setPosition(0);

        //  sleep(500);

        pozd=1.8;
        pozs=0;
        servo_dreapta.setPosition(pozd);
        servo_stanga.setPosition(pozs);

        sleep(500);

        while (opModeIsActive()) {

            double RoataStangaFata;
            double RoataDreaptaFata;
            double RoataStangaSpate;
            double RoataDreaptaSpate;


            double y1 = gamepad1.left_stick_y;
            double x1  =  gamepad1.left_stick_x;
            double x2  =  gamepad1.right_stick_x;

            PlatformaCuburi();
            Servouri();

            RoataStangaFata    = Range.clip(-x1 + y1 - x2, -0.5, 0.5) ;
            RoataDreaptaFata   = Range.clip(x1 + y1  + x2, -0.5, 0.5) ;
            RoataStangaSpate   = Range.clip(x1 + y1 + x2, -0.5, 0.5) ;
            RoataDreaptaSpate   = Range.clip(-x1 + y1 - x2, -0.5, 0.5) ;

            Roata_Stanga_Fata.setPower(RoataStangaFata);
            Roata_Stanga_Spate.setPower(RoataStangaSpate);
            Roata_Dreapta_Fata.setPower(RoataDreaptaFata);
            Roata_Dreapta_Spate.setPower(RoataDreaptaSpate);

            if(gamepad2.a){
                Brats.setPower(-1);
                a=1;
            }
            if(gamepad2.x){
                Bratd.setPower(-1);
                b=1;
            }
            if(gamepad2.b ){
                Brats.setPower(1);
                a=1;
            }
            if(gamepad2.y ){
                Bratd.setPower(1);
                b=1;
            }
            if(!gamepad2.a && !gamepad2.b && a==1) {
                Brats.setPower(0);
                a=0;
            }
            if(!gamepad2.x && !gamepad2.y && b==1) {
                Bratd.setPower(0);
                b=0;
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Stanga Fata (%.2f), Dreapta Fata (%.2f), Stanga Spate (%.2f), Dreapta Spate (%.2f)",
                    RoataStangaFata, RoataStangaFata, RoataStangaSpate, RoataDreaptaSpate);
            telemetry.update();


        }
    }


}