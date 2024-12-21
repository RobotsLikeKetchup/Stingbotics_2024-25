package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.hardware.DeadWheel;

public class Robot {
    //fields
    public DcMotor frontRight, armRotate, armExtend;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;

    public CRServo intakeRoller;
    public Servo intakeElbow, intakeClaw;

    //parallel dead wheels (measuring x-coord and heading)
    DeadWheel parL;
    DeadWheel parR;
    //perpendicular dead wheel (measuring y-coord)
    DeadWheel per;

    public TouchSensor slideLimitSwitch;
    public TouchSensor frontArmLimitSwitch;
    public TouchSensor backArmLimitSwitch;
    public DcMotor[] driveMotors;

    //constructor
    public Robot(DcMotor fR, DcMotor fL, DcMotor bR, DcMotor bL){
        frontRight = fR;
        frontLeft = fL;
        backRight = bR;
        backLeft = bL;
    }

    public Robot(){};

    public void init(HardwareMap hardwareMap){

        //TUNE THIS
        double inPerTick = 1;

        // Map variables to motors
        frontLeft = hardwareMap.get(DcMotor.class,"motor_fl");
        frontRight = hardwareMap.get(DcMotor.class, "motor_fr");
        backLeft = hardwareMap.get(DcMotor.class, "motor_bl");
        backRight = hardwareMap.get(DcMotor.class, "motor_br");

        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");

        intakeRoller = hardwareMap.get(CRServo.class, "intakeRoller");
        intakeElbow = hardwareMap.get(Servo.class, "intakeElbow");
        intakeClaw =  hardwareMap.get(Servo.class, "intakeClaw");

        slideLimitSwitch = hardwareMap.get(TouchSensor.class, "slideLimitSwitch");
        frontArmLimitSwitch = hardwareMap.get(TouchSensor.class, "frontArmLimitSwitch");
        backArmLimitSwitch = hardwareMap.get(TouchSensor.class, "backArmLimitSwitch");

        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

        // Set universal wheel behaviors
        for (DcMotor i : driveMotors) {
            i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            i.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        };
        armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //set deadwheel encoders
        parL = new DeadWheel(inPerTick, backRight);
        parR = new DeadWheel(inPerTick, frontRight);
        per = new DeadWheel(inPerTick, backLeft);

        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //methods
    public void setDeadwheels(DeadWheel p1, DeadWheel p2, DeadWheel pr){
        parL = p1;
        parR = p2;
        per = pr;
    }

    public DeadWheel getDeadwheel(String location){ // location can be parL, parR, or perp
        DeadWheel out;
        switch (location) {
            case "parL":
                out = parL;
                break;
            case "parR":
                out = parR;
                break;
            case "perp":
            case "per":
                out = per;
                break;
            default:
                out = null;
                break;
        }
        return out;
    }
}

