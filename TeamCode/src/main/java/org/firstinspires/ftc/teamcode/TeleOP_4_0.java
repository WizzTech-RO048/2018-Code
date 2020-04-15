package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "TeleOP 4.0")
public class TeleOP_4_0 extends LinearOpMode {

    //private Servo macara = null;
    //private Servo bratrelic = null;
    double pozs = 0;
    double pozd = 0;
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
    private Servo prindere_stanga = null;
    private Servo prindere_dreapta = null;

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
        //bratrelic = hardwareMap.get(Servo.class, "bratrelic");
        bratsecund = hardwareMap.get(Servo.class, "bratsecund");
        servo_stanga = hardwareMap.get(Servo.class, "servostanga");
        servo_dreapta = hardwareMap.get(Servo.class, "servodreapta");
        prindere_stanga = hardwareMap.get(Servo.class, "prindere_stanga");
        prindere_dreapta = hardwareMap.get(Servo.class, "prindere_dreapta");
        //macara = hardwareMap.get(Servo.class, "macara");
        bratmingi = hardwareMap.get(Servo.class, "bratmingi");

        Roata_Stanga_Fata.setDirection(DcMotor.Direction.REVERSE);
        Roata_Stanga_Spate.setDirection(DcMotor.Direction.REVERSE);
        Roata_Dreapta_Fata.setDirection(DcMotor.Direction.FORWARD);
        Roata_Dreapta_Spate.setDirection(DcMotor.Direction.FORWARD);
        Bratd.setDirection(DcMotor.Direction.FORWARD);
        Brats.setDirection(DcMotor.Direction.FORWARD);
        motorrelic.setDirection(DcMotor.Direction.FORWARD);
        prindere_stanga.setDirection(Servo.Direction.REVERSE);

        //servo_stanga.setDirection(Servo.Direction.REVERSE);
        //servo_dreapta.setDirection(Servo.Direction.REVERSE);

        bratmingi.setPosition(0.7);
        bratsecund.setPosition(1);
        servo_dreapta.setPosition(0.39);
        servo_stanga.setPosition(0.61);
        prindere_stanga.setPosition(0.15);
        prindere_dreapta.setPosition(0.15);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            boolean bratperpendicular;
            boolean brat_trigger = false;
            boolean d_pad = false;
            double RoataStangaFata;
            double RoataDreaptaFata;
            double RoataStangaSpate;
            double RoataDreaptaSpate;
            double y1 = -gamepad1.left_stick_y;
            double x1 = gamepad1.left_stick_x;
            double x2 = gamepad1.right_stick_x;
            int pozrelic = 2;
            //Miscare
            if (gamepad1.right_bumper == true) {
                RoataStangaFata = Range.clip(-x1 + y1 + x2, -1, 1);
                RoataDreaptaFata = Range.clip(x1 + y1 - x2, -1, 1);
                RoataStangaSpate = Range.clip(x1 + y1 + x2, -1, 1);
                RoataDreaptaSpate = Range.clip(-x1 + y1 - x2, -1, 1);

                if (gamepad1.dpad_up) {
                    d_pad = true;
                    Roata_Stanga_Fata.setPower(1);
                    Roata_Stanga_Spate.setPower(1);
                    Roata_Dreapta_Fata.setPower(1);
                    Roata_Dreapta_Spate.setPower(1);
                }
                if (gamepad1.dpad_left) {
                    d_pad = true;
                    Roata_Stanga_Fata.setPower(1);
                    Roata_Stanga_Spate.setPower(-1);
                    Roata_Dreapta_Fata.setPower(-1);
                    Roata_Dreapta_Spate.setPower(1);
                }
                if (gamepad1.dpad_right) {
                    d_pad = true;
                    Roata_Stanga_Fata.setPower(-1);
                    Roata_Stanga_Spate.setPower(1);
                    Roata_Dreapta_Fata.setPower(1);
                    Roata_Dreapta_Spate.setPower(-1);
                }
                if (gamepad1.dpad_down) {
                    d_pad = true;
                    Roata_Stanga_Fata.setPower(-1);
                    Roata_Stanga_Spate.setPower(-1);
                    Roata_Dreapta_Fata.setPower(-1);
                    Roata_Dreapta_Spate.setPower(-1);
                }
            } else {
                RoataStangaFata = Range.clip(-x1 + y1 + x2, -0.5, 0.5);
                RoataDreaptaFata = Range.clip(x1 + y1 - x2, -0.5, 0.5);
                RoataStangaSpate = Range.clip(x1 + y1 + x2, -0.5, 0.5);
                RoataDreaptaSpate = Range.clip(-x1 + y1 - x2, -0.5, 0.5);

                if (gamepad1.dpad_up) {
                    d_pad = true;
                    Roata_Stanga_Fata.setPower(0.3);
                    Roata_Stanga_Spate.setPower(0.3);
                    Roata_Dreapta_Fata.setPower(0.3);
                    Roata_Dreapta_Spate.setPower(0.3);
                }
                if (gamepad1.dpad_left) {
                    d_pad = true;
                    Roata_Stanga_Fata.setPower(0.3);
                    Roata_Stanga_Spate.setPower(-0.3);
                    Roata_Dreapta_Fata.setPower(-0.3);
                    Roata_Dreapta_Spate.setPower(0.3);
                }
                if (gamepad1.dpad_right) {
                    d_pad = true;
                    Roata_Stanga_Fata.setPower(-0.3);
                    Roata_Stanga_Spate.setPower(0.3);
                    Roata_Dreapta_Fata.setPower(0.3);
                    Roata_Dreapta_Spate.setPower(-0.3);
                }
                if (gamepad1.dpad_down) {
                    d_pad = true;
                    Roata_Stanga_Fata.setPower(-0.3);
                    Roata_Stanga_Spate.setPower(-0.3);
                    Roata_Dreapta_Fata.setPower(-0.3);
                    Roata_Dreapta_Spate.setPower(-0.3);
                }
            }

            if (gamepad1.dpad_down == false && gamepad1.dpad_right == false && gamepad1.dpad_left == false && gamepad1.dpad_up == false && d_pad == true) {
                d_pad = false;
                Roata_Stanga_Fata.setPower(0);
                Roata_Stanga_Spate.setPower(0);
                Roata_Dreapta_Fata.setPower(0);
                Roata_Dreapta_Spate.setPower(0);
            }

            if (d_pad != true) {
                Roata_Stanga_Fata.setPower(RoataStangaFata);
                Roata_Stanga_Spate.setPower(RoataStangaSpate);
                Roata_Dreapta_Fata.setPower(RoataDreaptaFata);
                Roata_Dreapta_Spate.setPower(RoataDreaptaSpate);
            }

            //Platforma cuburi
            if (gamepad2.dpad_up) {
                motorplatforma.setPower(1);
                pozd = 0.54;
                pozs = 0.46;
                servo_dreapta.setPosition(pozd);
                servo_stanga.setPosition(pozs);
            } else if (gamepad2.dpad_down) {
                motorplatforma.setPower(-1);
            } else {
                motorplatforma.setPower(0);
            }
            //Servouri
            if (gamepad2.y) {
                pozd = 0.85;
                pozs = 0.15;
                servo_dreapta.setPosition(pozd);
                servo_stanga.setPosition(pozs);
            }
            if (gamepad2.a) {
                pozd = 0.46;
                pozs = 0.54;
                servo_dreapta.setPosition(pozd);
                servo_stanga.setPosition(pozs);
            }
            //Roti cuburi -gata

            if (gamepad2.right_bumper) {
                Brats.setPower(-1);
                Bratd.setPower(1);
            } else if (gamepad2.left_bumper) {
                Brats.setPower(1);
                Bratd.setPower(-1);
            }
            if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0 && gamepad2.left_bumper == false && gamepad2.right_bumper == false) {
                Brats.setPower(0);
                Bratd.setPower(0);
            }
            /*if(gamepad2.left_trigger!=0) {
                Brats.setPower(gamepad2.left_trigger);
            }
            else if(gamepad2.left_bumper==false && gamepad2.right_bumper==false){
                Brats.setPower(0);
            }
            if(gamepad2.right_trigger!=0) {
                Bratd.setPower(-gamepad2.right_trigger);
            }
            else if(gamepad2.left_bumper==false && gamepad2.right_bumper==false){
                Bratd.setPower(0);
            }*/
            if (gamepad2.left_trigger != 0.0 || gamepad2.right_trigger != 0.0) {
                Brats.setPower(gamepad2.left_trigger);
                Bratd.setPower(-gamepad2.right_trigger);
            } else if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
                Brats.setPower(0);
                Bratd.setPower(0);
            }

            //RELIC
            if (gamepad1.left_trigger != 0) {
                motorrelic.setPower(gamepad1.left_trigger);

            } else if (gamepad1.right_trigger != 0) {
                motorrelic.setPower(-gamepad1.right_trigger);
            } else {
                motorrelic.setPower(0);
            }

            /*if (gamepad1.y) {
                macara.setPosition(0.8);
            }
            if (gamepad1.b) {
                macara.setPosition(0.0);
            }

            if (gamepad1.a) {
                bratrelic.setPosition(0.34);
            }
            if (gamepad1.x) {
                //poz default
                    bratrelic.setPosition(0.67);
                    //bratrelic.setPosition(0);
            }

            if (gamepad1.left_bumper) {
                bratrelic.setPosition(0.0);
            }*/

            //PRindere
            if (gamepad2.dpad_left) {
                prindere_stanga.setPosition(0.3);
                prindere_dreapta.setPosition(0.3);
            }

            if (gamepad2.dpad_right) {
                prindere_stanga.setPosition(0.15);
                prindere_dreapta.setPosition(0.15);
            }
            //Default bratZ
            if (gamepad2.left_stick_button) {
                bratmingi.setPosition(0.7);
                bratsecund.setPosition(1);
            }


            telemetry.addData("Speed", x1 + x2 + y1);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Stanga Fata (%.2f), Dreapta Fata (%.2f), Stanga Spate (%.2f), Dreapta Spate (%.2f)",
                    RoataStangaFata, RoataStangaFata, RoataStangaSpate, RoataDreaptaSpate);
            telemetry.update();

        }
    }


}