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


@TeleOp(name="TeleOP")
public class TeleOP extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Roata_Stanga_Fata = null;
    private DcMotor Roata_Stanga_Spate = null;
    private DcMotor Roata_Dreapta_Fata = null;
    private DcMotor Roata_Dreapta_Spate = null;
    private DcMotor Brats = null;
    private DcMotor Bratd = null;
    private DcMotor motorplatforma = null;
    private Servo servo_stanga=null;
    private Servo servo_dreapta=null;
    private Servo bratsecund=null;
    private CRServo brat=null;
    private CRServo bratroti=null;
    double  pozs = 0;
    double pozd=0;
    double pozclasics=0.68;
    double pozclasicd=0.3;
    double a;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Roata_Stanga_Fata = hardwareMap.get(DcMotor.class, "sf");
        Roata_Stanga_Spate = hardwareMap.get(DcMotor.class, "ss");
        Roata_Dreapta_Fata = hardwareMap.get(DcMotor.class, "df");
        Roata_Dreapta_Spate = hardwareMap.get(DcMotor.class, "ds");
        bratsecund = hardwareMap.get(Servo.class, "bratsecund");
        brat = hardwareMap.get(CRServo.class, "bratmingi");
        bratroti=hardwareMap.get(CRServo.class,"bratroti");
        brat.setPower(0);
        bratroti.setPower(0);
        bratsecund.setPosition(1);
        brat.setDirection(DcMotorSimple.Direction.FORWARD);
        brat.setPower(0.3);

        Brats = hardwareMap.get(DcMotor.class, "brats");
        Bratd = hardwareMap.get(DcMotor.class, "bratd");
        motorplatforma = hardwareMap.get(DcMotor.class, "platforma");
        servo_stanga = hardwareMap.get(Servo.class, "servo_stanga");
        servo_dreapta = hardwareMap.get(Servo.class, "servo_dreapta");
        brat.resetDeviceConfigurationForOpMode();
        bratsecund.resetDeviceConfigurationForOpMode();
        bratroti.resetDeviceConfigurationForOpMode();

        Roata_Stanga_Fata.setDirection(DcMotor.Direction.REVERSE);
        Roata_Stanga_Spate.setDirection(DcMotor.Direction.REVERSE);
        Roata_Dreapta_Fata.setDirection(DcMotor.Direction.FORWARD);
        Roata_Dreapta_Spate.setDirection(DcMotor.Direction.FORWARD);
        Bratd.setDirection(DcMotor.Direction.FORWARD);
        Brats.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {

            double RoataStangaFata;
            double RoataDreaptaFata;
            double RoataStangaSpate;
            double RoataDreaptaSpate;



            double y1 = gamepad1.left_stick_y;
            double x1  =  gamepad1.left_stick_x;
            double x2  =  gamepad1.right_stick_x;
            //platforma cuburi
            if (gamepad2.x) {
                motorplatforma.setPower(1);
                pozs=0.56;
                pozd=0.42;
                servo_dreapta.setPosition(pozd);
                servo_stanga.setPosition(pozs);
            }
            else if(gamepad2.y) {
                motorplatforma.setPower(-1);
            }
            else {
                motorplatforma.setPower(0);
            }
            //SERVOURI
            if (gamepad2.a) {
                pozd=1;
                pozs=0;
                servo_dreapta.setPosition(pozd);
                servo_stanga.setPosition(pozs);
            }
            if (gamepad2.b) {
                pozd=0.32;
                pozs=0.66;
                servo_dreapta.setPosition(pozd);
                servo_stanga.setPosition(pozs);
            }

            if(gamepad1.a){
                Bratd.setPower(-gamepad1.right_trigger);
                Brats.setPower(-gamepad1.left_trigger);
            }
            if (gamepad1.x){
                Bratd.setPower(0.2);
                Brats.setPower(1);
            }
            if(gamepad1.y)
            {
                bratroti.setDirection(DcMotorSimple.Direction.REVERSE);
                bratroti.setPower(1);
            }
            else {
                bratroti.setPower(0);
            }

            RoataStangaFata    = Range.clip(x1 + y1 + x2, -0.5, 0.5) ;
            RoataDreaptaFata   = Range.clip(-x1 + y1  - x2, -0.5, 0.5) ;
            RoataStangaSpate   = Range.clip(-x1 + y1 + x2, -0.5, 0.5) ;
            RoataDreaptaSpate   = Range.clip(x1 + y1 - x2, -0.5, 0.5) ;

            Roata_Stanga_Fata.setPower(RoataStangaFata);
            Roata_Stanga_Spate.setPower(RoataStangaSpate);
            Roata_Dreapta_Fata.setPower(RoataDreaptaFata);
            Roata_Dreapta_Spate.setPower(RoataDreaptaSpate);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Stanga Fata (%.2f), Dreapta Fata (%.2f), Stanga Spate (%.2f), Dreapta Spate (%.2f)",
                    RoataStangaFata, RoataStangaFata, RoataStangaSpate, RoataDreaptaSpate);
            telemetry.update();

        }
    }


}