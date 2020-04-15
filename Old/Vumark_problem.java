package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.VuMarkTarget;
import com.vuforia.VuMarkTargetResult;
import com.vuforia.VuMarkTemplate;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.vuforia.CameraDevice;
import com.vuforia.Vuforia;

import java.util.concurrent.TimeUnit;


@Autonomous(name = "Vumark problem")
public class Vumark_problem extends LinearOpMode {
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
        waitForStart();
        runtime.reset();
        int col = ceva();

    }


    VuforiaLocalizer vuforia;

    public int ceva() {

        int col = -1;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AVR36zX/////AAAAmf4wKl7jz04GkSKCzFf6A/gIfIqXUXFiD8Q8smknNL1/7mQ7bfNDDX/GKq/vejsAFCypbnHQwmHunWd2pk/dOZY2Wi4Nj64UbWmDawkA3Jy9oiIfS4C7sfViotjeuhQZuuUuHOEDh63tJEX54rWgXUHYYpBUApz+2pB4ijjg+YipO05M9EEInFeUqL3rpEu1vuLq942L/tc7r2C/Am9W37dHlCMlIBYJ2f1RpTdi/6+WVFuiD5lhIyL6hGU9OMhmRzBrZeJWF0GNFHqi21JxiD9IpRIsk6KDqMdS/gYEWg20Lkk/vPEDbKXCrdm9iAQjlBAzjPq0gIE6i785JI8y61qzxohgI4PXEF0ciUqiC2UD";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        CameraDevice.getInstance().setField("exposure-time","1/15");
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

    }}