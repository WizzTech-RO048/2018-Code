package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Albastru - aproape")
public class sad extends LinearOpMode {

    private static double a_1 = 62;
    private static double DISTANCE_BETWEEN_CRYPTOBOX_SLTOS = 19;

    private static PredefinedColor PlatformColor = PredefinedColor.BLUE;

    //CONSTANTS
    private int minColorValue = 30;
    private int AngleZ;
    private boolean finishColor = false;
    private VuforiaLocalizer vuforia;
    private RelicRecoveryVuMark vuMark = null;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    private boolean vuforiaFound = false;

    private ModernRoboticsI2cGyro gyro;

    private robot r = new robot();

    @Override
    public void runOpMode() throws InterruptedException {
        r.initRobot(hardwareMap);
        initVuforia();
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        gyro.calibrate();
        gyro.resetZAxisIntegrator();

        a_1 = 62;

        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        waitForStart();

//        colorCheck();
//        checkVuforia();
//
//        while (!vuforiaFound() && !finishColor && opModeIsActive()) {
//        }
//
//        switch (vuMark) {
//            case CENTER:
//                a_1 += DISTANCE_BETWEEN_CRYPTOBOX_SLTOS;
//                break;
//            case RIGHT:
//                a_1 += DISTANCE_BETWEEN_CRYPTOBOX_SLTOS + DISTANCE_BETWEEN_CRYPTOBOX_SLTOS;
//                break;
//        }

        encoderDrive(0.1, -a_1);
        turnTo(0.1, -88);


        r.getBrats().setPower(-1);
        r.getBratd().setPower(1);
        TimeUnit.MILLISECONDS.sleep(2000);
        r.getBrats().setPower(0);
        r.getBratd().setPower(0);

        encoderDrive(0.1, -15);

        r.getServo_stanga().setPosition(0.15);
        r.getServo_dreapta().setPosition(0.85);
        TimeUnit.MILLISECONDS.sleep(1000);
        r.getServo_stanga().setPosition(0.58);
        r.getServo_dreapta().setPosition(0.42);

        encoderDrive(0.1, 23);
    }

    @SuppressWarnings("deprecation")
    private void turnTo(double power, int degrees) throws InterruptedException {

        AngleZ = gyro.getIntegratedZValue();
        while (Math.abs(AngleZ - degrees) > 0) {
            if (AngleZ > degrees) {
                Dreapta(power);
            } else if (AngleZ < degrees) {
                Stanga(power);
            }
            waitOneFullHardwareCycle();
            AngleZ = gyro.getIntegratedZValue();
            telemetry.addData("Z", AngleZ);
            telemetry.update();
        }
        waitOneFullHardwareCycle();
        Stop();
    }

    private boolean vuforiaFound() {
        return vuforiaFound;
    }

    private void checkVuforia() {
        relicTrackables.activate();

        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while (opModeIsActive() && vuMark.equals(RelicRecoveryVuMark.UNKNOWN)) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark:", vuMark);
            telemetry.update();
        }

        vuforiaFound = true;

    }

    private void colorCheck() {

        PredefinedColor BallColor;

        goColorServoHandTarget();

        sleep(1000);

        BallColor = getColor(r.getColorSensor().readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));

        if (!BallColor.equals(PredefinedColor.UNKNOWN) && !PlatformColor.equals(PredefinedColor.UNKNOWN))
            if (BallColor.equals(PlatformColor)) {
                //ROTIRE STANGA
            } else {
                //ROTIRE DREAPTA
            }

        goColorServoHandHome();

        finishColor = true;
    }

    private void goColorServoHandHome() {
        r.getBrat().setPosition(0.78);
        r.getBratsecund().setPosition(1);
    }

    private void goColorServoHandTarget() {
        r.getBrat().setPosition(0);
        r.getBratsecund().setPosition(0);
    }

    private PredefinedColor getColor(int color) {
        if (color == 3) return PredefinedColor.RED;
        else if (color == 10) return PredefinedColor.BLUE;
        else return PredefinedColor.UNKNOWN;
    }

    public void encoderDrive(double speed, double cm) {

        double inch = cm * (1 / 2.54);

        int TICKS_PER_REVOLUTION = 1120;
        double COUNTS_PER_INCH = TICKS_PER_REVOLUTION / 16.5;

        r.getRoata_Stanga_Fata().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.getRoata_Dreapta_Fata().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.getRoata_Stanga_Spate().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.getRoata_Dreapta_Spate().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        r.getRoata_Stanga_Fata().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.getRoata_Dreapta_Fata().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.getRoata_Stanga_Spate().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.getRoata_Dreapta_Spate().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        r.getRoata_Stanga_Fata().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.getRoata_Dreapta_Fata().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.getRoata_Stanga_Spate().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.getRoata_Dreapta_Spate().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newPosition = (int) (COUNTS_PER_INCH * inch);

        r.getRoata_Stanga_Fata().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.getRoata_Dreapta_Fata().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.getRoata_Stanga_Spate().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.getRoata_Dreapta_Spate().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        r.getRoata_Stanga_Fata().setTargetPosition(-newPosition);
        r.getRoata_Dreapta_Fata().setTargetPosition(newPosition);
        r.getRoata_Stanga_Spate().setTargetPosition(newPosition);
        r.getRoata_Dreapta_Spate().setTargetPosition(-newPosition);

        r.getRoata_Stanga_Fata().setPower(-speed);
        r.getRoata_Dreapta_Fata().setPower(speed);
        r.getRoata_Stanga_Spate().setPower(speed);
        r.getRoata_Dreapta_Spate().setPower(-speed);

        while (r.getRoata_Stanga_Fata().isBusy() || r.getRoata_Dreapta_Fata().isBusy() || r.getRoata_Stanga_Spate().isBusy() || r.getRoata_Dreapta_Spate().isBusy()) {
            telemetry.addData("LMU", r.getRoata_Stanga_Fata().getCurrentPosition());
            telemetry.addData("RMU", r.getRoata_Dreapta_Fata().getCurrentPosition());
            telemetry.addData("LMD", r.getRoata_Stanga_Spate().getCurrentPosition());
            telemetry.addData("RMD", r.getRoata_Dreapta_Spate().getCurrentPosition());
            telemetry.update();
        }

        r.getRoata_Stanga_Fata().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.getRoata_Dreapta_Fata().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.getRoata_Stanga_Spate().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.getRoata_Dreapta_Spate().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        r.getRoata_Stanga_Fata().setPower(0.0);
        r.getRoata_Dreapta_Fata().setPower(0.0);
        r.getRoata_Stanga_Spate().setPower(0.0);
        r.getRoata_Dreapta_Spate().setPower(0.0);

    }

    private PredefinedColor getColor(int R, int B) {
        if (R > B) return R > minColorValue ? PredefinedColor.RED : PredefinedColor.UNKNOWN;
        else if (B > R) return B > minColorValue ? PredefinedColor.BLUE : PredefinedColor.UNKNOWN;
        else return PredefinedColor.UNKNOWN;
    }

    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ae1B0KT/////AAAAmb1XYh8lnkJApASHU4GlfoqG1HM2p/vcZ5IxoIMZChOo2PH0w70nDnGNquAykLVE1+9dA+dH8LGl5G1s0ts72YIhfH7FShO4GtIjsIkf8Sgolfi3qdzfQ+t0ga1a90ISGY3ZxKFoz6M6I8URFSPwju493j1WM73/xTwIWyMy3SSgz8O0S+MSrYTUG8e97iY3RLcH6OefPQNWzvH9Lh8+rxnjwR9RR40WHD/Oefh83kN7EanocJi/PUxTc+zAlfrcurVQCUTOd3yHlZeFtrZ9zVMgPZ/p9RKYK+/gUKYmmdBALrtjkFC6YI6XPRgCUnVZU9QP6DWj7XKT93PDRlaSvmhBztDG+GGGb9/Vu2Hwbg5b";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }

    public void Dreapta(double speed) {
        if (opModeIsActive()) {
            r.getRoata_Stanga_Fata().setPower(-speed);
            r.getRoata_Dreapta_Fata().setPower(-speed);
            r.getRoata_Stanga_Spate().setPower(-speed);
            r.getRoata_Dreapta_Spate().setPower(-speed);
        }
    }

    public void Stanga(double speed) {
        if (opModeIsActive()) {
            r.getRoata_Stanga_Fata().setPower(speed);
            r.getRoata_Dreapta_Fata().setPower(speed);
            r.getRoata_Stanga_Spate().setPower(speed);
            r.getRoata_Dreapta_Spate().setPower(speed);
        }
        //stop();
    }

    public void Corectare(double directie, int i) {

        while (opModeIsActive() && gyro.getIntegratedZValue() < directie) {
            if (gyro.getIntegratedZValue() < directie) {
                Dreapta(0.2);
            }
            Dreapta(0.2);
            if (gyro.getIntegratedZValue() > directie) {
                Stanga(0.2);
            }
        }

        Stop();

    }

    public void Stop() {
        if (opModeIsActive()) {
            r.getRoata_Stanga_Fata().setPower(0.0);
            r.getRoata_Dreapta_Fata().setPower(0.0);
            r.getRoata_Stanga_Spate().setPower(0.0);
            r.getRoata_Dreapta_Spate().setPower(0.0);
        }
    }
}