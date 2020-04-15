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


@TeleOp(name="TeleOP 2.0")
public class TeleOP_2_0 extends LinearOpMode {

    public static class Initialization {
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
    }
    Initialization robot = new Initialization();
    public final double COUNTS_PER_MOTOR_REV = 1440;
    public final double WHEEL_DIAMETER_INCHES = 10.16;
    public final double COUNTS_PER_CM = COUNTS_PER_MOTOR_REV /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.Roata_Stanga_Fata = hardwareMap.get(DcMotor.class, "sf");
        robot.Roata_Stanga_Spate = hardwareMap.get(DcMotor.class, "ss");
        robot.Roata_Dreapta_Fata = hardwareMap.get(DcMotor.class, "df");
        robot.Roata_Dreapta_Spate = hardwareMap.get(DcMotor.class, "ds");
        robot.bratsecund = hardwareMap.get(Servo.class, "bratsecund");
        robot.brat = hardwareMap.get(CRServo.class, "bratmingi");
        robot.bratroti=hardwareMap.get(CRServo.class,"bratroti");
        robot.Brats = hardwareMap.get(DcMotor.class, "brats");
        robot.Bratd = hardwareMap.get(DcMotor.class, "bratd");
        robot.motorplatforma = hardwareMap.get(DcMotor.class, "platforma");
        robot.servo_stanga = hardwareMap.get(Servo.class, "servo_stanga");
        robot.servo_dreapta = hardwareMap.get(Servo.class, "servo_dreapta");
        robot.brat.resetDeviceConfigurationForOpMode();
        robot.bratsecund.resetDeviceConfigurationForOpMode();
        robot.bratroti.resetDeviceConfigurationForOpMode();
        robot.Roata_Stanga_Fata.setDirection(DcMotor.Direction.REVERSE);
        robot.Roata_Stanga_Spate.setDirection(DcMotor.Direction.REVERSE);
        robot.Roata_Dreapta_Fata.setDirection(DcMotor.Direction.FORWARD);
        robot.Roata_Dreapta_Spate.setDirection(DcMotor.Direction.FORWARD);
        robot.Bratd.setDirection(DcMotor.Direction.FORWARD);
        robot.Brats.setDirection(DcMotor.Direction.FORWARD);
        //initializare pozitii brate ridicate
        robot.brat.setPower(0);
        robot.bratroti.setPower(0);
        robot.bratsecund.setPosition(1);
        robot.brat.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.brat.setPower(0.3);
        //

        waitForStart();
        while (opModeIsActive()) {
            double RoataStangaFata;
            double RoataDreaptaFata;
            double RoataStangaSpate;
            double RoataDreaptaSpate;
            double y1 = gamepad1.left_stick_y;
            double x1  =  gamepad1.left_stick_x;
            double x2  =  gamepad1.right_stick_x;

            //MISCARE ROBOT
            RoataStangaFata    = Range.clip(x1 + y1 + x2, -0.5, 0.5) ;
            RoataDreaptaFata   = Range.clip(-x1 + y1  - x2, -0.5, 0.5) ;
            RoataStangaSpate   = Range.clip(-x1 + y1 + x2, -0.5, 0.5) ;
            RoataDreaptaSpate   = Range.clip(x1 + y1 - x2, -0.5, 0.5) ;
            robot.Roata_Stanga_Fata.setPower(RoataStangaFata);
            robot.Roata_Stanga_Spate.setPower(RoataStangaSpate);
            robot.Roata_Dreapta_Fata.setPower(RoataDreaptaFata);
            robot.Roata_Dreapta_Spate.setPower(RoataDreaptaSpate);

            //lift platforma cuburi
            if (gamepad2.x) {
                robot.motorplatforma.setPower(1);
                robot.pozs=0.56;

                robot.pozd=0.42;
                robot.servo_dreapta.setPosition(robot.pozd);
                robot.servo_stanga.setPosition(robot.pozs);
            }
            else if(gamepad2.y) {
                robot.motorplatforma.setPower(-1);
            }
            else {
                robot.motorplatforma.setPower(0);
            }
            //SERVOURI PLATFORMA
            if (gamepad2.a) {
                robot.pozd=1;
                robot.pozs=0;
                robot.servo_dreapta.setPosition(robot.pozd);
                robot.servo_stanga.setPosition(robot.pozs);
            }
            if (gamepad2.b) {
                robot.pozd=0.32;
                robot.pozs=0.66;
                robot.servo_dreapta.setPosition(robot.pozd);
                robot.servo_stanga.setPosition(robot.pozs);
            }
            //ROTI VERZI
            if(gamepad1.a){
                robot.Bratd.setPower(-gamepad1.right_trigger);
                robot.Brats.setPower(-gamepad1.left_trigger);
            }
            if (gamepad1.x){
                robot.Bratd.setPower(0.2);
                robot.Brats.setPower(1);
            }
            //REV SERVO ROTI VERZI
            if(gamepad1.y)
            {
                robot.bratroti.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.bratroti.setPower(1);
            }
            else {
                robot.bratroti.setPower(0);
            }

            //TELEMETRY
            telemetry.addData("Status", "Run Time: " + robot.runtime.toString());
            telemetry.addData("Motors", "Stanga Fata (%.2f), Dreapta Fata (%.2f), Stanga Spate (%.2f), Dreapta Spate (%.2f)",
                    RoataStangaFata, RoataStangaFata, RoataStangaSpate, RoataDreaptaSpate);
            telemetry.update();

        }
    }


}