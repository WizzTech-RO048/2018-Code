package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class robot {

    private DcMotor Roata_Stanga_Fata, Roata_Stanga_Spate, Roata_Dreapta_Fata, Roata_Dreapta_Spate, Brats, Bratd;
    private Servo brat, bratsecund, servo_stanga, servo_dreapta;
    private ModernRoboticsI2cColorSensor colorSensor;

    private HardwareMap hardwareMap;

    public robot() {
    }

    public void initRobot(HardwareMap hardwareMap) {

        Roata_Stanga_Fata = hardwareMap.dcMotor.get("ss");
        Roata_Dreapta_Fata = hardwareMap.dcMotor.get("ds");
        Roata_Stanga_Spate = hardwareMap.dcMotor.get("sf");
        Roata_Dreapta_Spate = hardwareMap.dcMotor.get("df");

        Roata_Stanga_Fata.setDirection(DcMotorSimple.Direction.REVERSE);
        Roata_Dreapta_Fata.setDirection(DcMotorSimple.Direction.REVERSE);
        Roata_Stanga_Spate.setDirection(DcMotorSimple.Direction.REVERSE);
        Roata_Dreapta_Spate.setDirection(DcMotorSimple.Direction.REVERSE);

        Brats = hardwareMap.dcMotor.get("brats");
        Bratd = hardwareMap.dcMotor.get("bratd");

        brat = hardwareMap.servo.get("bratmingi");
        bratsecund = hardwareMap.servo.get("bratsecund");
        servo_stanga = hardwareMap.servo.get("servostanga");
        servo_dreapta = hardwareMap.servo.get("servodreapta");

        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "jewels_sensor");

        this.hardwareMap = hardwareMap;
    }

    public DcMotor getRoata_Stanga_Fata() {
        return Roata_Stanga_Fata;
    }

    public DcMotor getRoata_Stanga_Spate() {
        return Roata_Stanga_Spate;
    }

    public DcMotor getRoata_Dreapta_Fata() {
        return Roata_Dreapta_Fata;
    }

    public DcMotor getRoata_Dreapta_Spate() {
        return Roata_Dreapta_Spate;
    }

    public DcMotor getBrats() {
        return Brats;
    }

    public DcMotor getBratd() {
        return Bratd;
    }

    public Servo getBrat() {
        return brat;
    }

    public Servo getBratsecund() {
        return bratsecund;
    }

    public Servo getServo_stanga() {
        return servo_stanga;
    }

    public Servo getServo_dreapta() {
        return servo_dreapta;
    }

    public ModernRoboticsI2cColorSensor getColorSensor() {
        return colorSensor;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }
}