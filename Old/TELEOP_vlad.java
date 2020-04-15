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


@TeleOp(name="TeleOp Vlad")
public class TELEOP_vlad   extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Roata_Stanga_Fata = null;
    private DcMotor Roata_Stanga_Spate = null;
    private DcMotor Roata_Dreapta_Fata = null;
    private DcMotor Roata_Dreapta_Spate = null;
    private DcMotor Brats = null;
    private DcMotor Bratd = null;
    private DcMotor motorplatforma = null;
    private DcMotor motorrelic=null;
    //static final int    CYCLE_MS    =   50;     // period of each cycle
    Servo servo_stanga;
    Servo servo_dreapta;
    CRServo bratroti;
    CRServo bratmingi;
    Servo bratsecund;
    double  pozs = 0;
    double pozd=0;
    double pozclasics=0.66;
    double pozclasicd=0.32;
    public void PlatformaCuburi() {

        if (gamepad1.x) {
            motorplatforma.setPower(1);
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

    }
    public void Servouri()
    {
        if (gamepad1.a) {

            pozd=1;
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

    public void RotiCuburi()
    {
        if(gamepad1.right_bumper) {
            Brats.setPower(-0.3);
            Bratd.setPower(-0.3);
        }
        else if(gamepad1.left_bumper) {


            Brats.setPower(1);
            Bratd.setPower(1);
        }
        else
        {
            Brats.setPower(0);
            Bratd.setPower(0);
        }
    }
    /*public void ServoRoti() throws InterruptedException
    {
        if(gamepad2.y) {
            bratroti.setDirection(DcMotorSimple.Direction.REVERSE);
            bratroti.setPower(1);
            TimeUnit.SECONDS.sleep(3);
            bratroti.setPower(0);
        }
        else if(gamepad2.x)
        {
            bratroti.setDirection(DcMotorSimple.Direction.FORWARD);
            bratroti.setPower(1);
            TimeUnit.SECONDS.sleep(3);
            bratroti .setPower(0);
        }
        else
            bratroti.setPower(0);
    }
    */
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
        motorrelic=hardwareMap.get(DcMotor.class,"motorrelic");
        servo_stanga = hardwareMap.get(Servo.class, "servo_stanga");
        servo_dreapta = hardwareMap.get(   Servo.class, "servo_dreapta");
        bratmingi=hardwareMap.get(CRServo.class,"bratmingi");
        bratsecund=hardwareMap.get(Servo.class,"bratsecund");
        bratroti=hardwareMap.get(CRServo.class,"bratroti");
        Roata_Stanga_Fata.setDirection(DcMotor.Direction.REVERSE);
        Roata_Stanga_Spate.setDirection(DcMotor.Direction.REVERSE);
        Roata_Dreapta_Fata.setDirection(DcMotor.Direction.FORWARD);
        Roata_Dreapta_Spate.setDirection(DcMotor.Direction.FORWARD);
        Bratd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Brats.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Bratd.setDirection(DcMotor.Direction.FORWARD);
        Brats.setDirection(DcMotor.Direction.REVERSE);

        bratroti.setPower(0);
        bratmingi.setPower(0.3);
        bratsecund.setPosition(1.8);
        waitForStart();
        runtime.reset();



        while (opModeIsActive()) {

            double RoataStangaFata;
            double RoataDreaptaFata;
            double RoataStangaSpate;
            double RoataDreaptaSpate;
            double y1 = -gamepad1.left_stick_y;
            double x1  =  gamepad1.left_stick_x;
            double x2  =  gamepad1.right_stick_x;

            RoataStangaFata    = Range.clip(x1 + y1 + x2, -0.5, 0.5) ;
            RoataDreaptaFata   = Range.clip(-x1 + y1  - x2, -0.5, 0.5) ;
            RoataStangaSpate   = Range.clip(-x1 + y1 + x2, -0.5, 0.5) ;
            RoataDreaptaSpate   = Range.clip(x1 + y1 - x2, -0.5, 0.5) ;
            Roata_Stanga_Fata.setPower(RoataStangaFata);
            Roata_Stanga_Spate.setPower(RoataStangaSpate);
            Roata_Dreapta_Fata.setPower(RoataDreaptaFata);
            Roata_Dreapta_Spate.setPower(RoataDreaptaSpate);

            RotiCuburi();
            PlatformaCuburi();
            Servouri();


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Stanga Fata (%.2f), Dreapta Fata (%.2f), Stanga Spate (%.2f), Dreapta Spate (%.2f)",
                    RoataStangaFata, RoataStangaFata, RoataStangaSpate, RoataDreaptaSpate);
            telemetry.update();

        }
    }


}