package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "TeleOP 3.0")
public class TeleOP_3_0 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Roata_Stanga_Fata = null;
    private DcMotor Roata_Stanga_Spate = null;
    private DcMotor Roata_Dreapta_Fata = null;
    private DcMotor Roata_Dreapta_Spate = null;
    private DcMotor Brats = null;
    private DcMotor Bratd = null;
    private DcMotor motorplatforma = null;
    private DcMotor motorrelic = null;
    private Servo servo_stanga = null;
    private Servo servo_dreapta = null;
    private Servo bratsecund = null;
    private Servo bratmingi = null;
    double pozs = 0;
    double pozd = 0;
    //double pozclasics = 0.68;
    //double pozclasicd = 0.3;

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
        motorrelic = hardwareMap.get(DcMotor.class, "motorrelic");

        bratsecund = hardwareMap.get(Servo.class, "bratsecund");
        servo_stanga = hardwareMap.get(Servo.class, "servostanga");
        servo_dreapta = hardwareMap.get(Servo.class, "servodreapta");

        bratmingi = hardwareMap.get(Servo.class, "bratmingi");

        Roata_Stanga_Fata.setDirection(DcMotor.Direction.REVERSE);
        Roata_Stanga_Spate.setDirection(DcMotor.Direction.REVERSE);
        Roata_Dreapta_Fata.setDirection(DcMotor.Direction.FORWARD);
        Roata_Dreapta_Spate.setDirection(DcMotor.Direction.FORWARD);
        Bratd.setDirection(DcMotor.Direction.FORWARD);
        Brats.setDirection(DcMotor.Direction.FORWARD);
        //servo_stanga.setDirection(Servo.Direction.REVERSE);
        //servo_dreapta.setDirection(Servo.Direction.REVERSE);

        bratmingi.setPosition(1);
        bratsecund.setPosition(1);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double RoataStangaFata;
            double RoataDreaptaFata;
            double RoataStangaSpate;
            double RoataDreaptaSpate;
            double y1 = -gamepad1.left_stick_y;
            double x1 = gamepad1.left_stick_x;
            double x2 = gamepad1.right_stick_x;
            boolean r = false;

            //Miscare
            RoataStangaFata = Range.clip(-x1 + y1 + x2, -0.5, 0.5);
            RoataDreaptaFata = Range.clip(x1 + y1 - x2, -0.5, 0.5);
            RoataStangaSpate = Range.clip(x1 + y1 + x2, -0.5, 0.5);
            RoataDreaptaSpate = Range.clip(-x1 + y1 - x2, -0.5, 0.5);
            /*
            if(r==false) {
                if(gamepad1.dpad_right) {
                    r = true;
                }
                RoataStangaFata    = Range.clip(x1 + y1 + x2, -0.5, 0.5) ;
                RoataDreaptaFata   = Range.clip(-x1 + y1  - x2, -0.5, 0.5) ;
                RoataStangaSpate   = Range.clip(-x1 + y1 + x2, -0.5, 0.5) ;
                RoataDreaptaSpate   = Range.clip(x1 + y1 - x2, -0.5, 0.5) ;
            }
            else
            {
                if(gamepad1.dpad_right) {
                    r = false;
                }
                RoataStangaFata    = Range.clip(x1 + y1 + x2, -1, 1) ;
                RoataDreaptaFata   = Range.clip(-x1 + y1  - x2, -1, 1) ;
                RoataStangaSpate   = Range.clip(-x1 + y1 + x2, -1, 1) ;
                RoataDreaptaSpate   = Range.clip(x1 + y1 - x2, -1, 1) ;
            }
            */

            Roata_Stanga_Fata.setPower(RoataStangaFata);
            Roata_Stanga_Spate.setPower(RoataStangaSpate);
            Roata_Dreapta_Fata.setPower(RoataDreaptaFata);
            Roata_Dreapta_Spate.setPower(RoataDreaptaSpate);

            //Platforma cuburi
            if (gamepad1.x) {
                motorplatforma.setPower(1);
                pozd=0.43;
                pozs=0.57;
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
            //Servouri
            if (gamepad1.a) {
                pozd=0.82;
                pozs=0.18;
                servo_dreapta.setPosition(pozd);
                servo_stanga.setPosition(pozs);
            }
            if (gamepad1.b) {
                pozd=0.39;
                pozs=0.61;
                servo_dreapta.setPosition(pozd);
                servo_stanga.setPosition(pozs);
            }
            //Roti cuburi -gata
            if (gamepad2.left_bumper) {
                Brats.setPower(-1);
            } else {
                Brats.setPower(gamepad2.left_trigger);
            }
            if (gamepad2.right_bumper) {
                Bratd.setPower(1);
            } else {
                Bratd.setPower(-gamepad2.right_trigger);
            }


            telemetry.addData("Speed", x1 + x2 + y1);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Stanga Fata (%.2f), Dreapta Fata (%.2f), Stanga Spate (%.2f), Dreapta Spate (%.2f)",
                    RoataStangaFata, RoataStangaFata, RoataStangaSpate, RoataDreaptaSpate);
            telemetry.update();

        }
    }


}