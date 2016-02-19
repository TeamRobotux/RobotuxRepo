package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
public class RobotuxAuto_AD extends LinearOpMode {
    final static double TRAY_MIN_RANGE  = 1.0;
    final static double TRAY_MAX_RANGE  = 0.1;
    double colorvalue;

    DcMotor rightRear = null;
    DcMotor leftRear = null;
    DcMotor rightFront = null;
    DcMotor leftFront = null;
    DcMotor lift = null;
    DcMotor tilt = null;
    DcMotor pickUp = null;
    Servo leftLever = null;
    Servo rightLever = null;
    Servo tray = null;
    //ColorSensor csensor = null;

    double trayPosition = 1.0;
    public void runOpMode() throws InterruptedException {

        // Initialize motors
        rightRear = hardwareMap.dcMotor.get("motor_2b");
        leftRear = hardwareMap.dcMotor.get("motor_1b");
        rightFront = hardwareMap.dcMotor.get("motor_2f");
        leftFront = hardwareMap.dcMotor.get("motor_1f");
        pickUp = hardwareMap.dcMotor.get("flipper");
        lift = hardwareMap.dcMotor.get("motorLift");
        tilt = hardwareMap.dcMotor.get("motorTilt");
        leftLever=hardwareMap.servo.get("motor_leftL");
        rightLever=hardwareMap.servo.get("motor_rightL");
        tray = hardwareMap.servo.get("motor_tray");
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        //csensor =hardwareMap.colorSensor.get("colorsensor");

        // set the mode
        rightRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        leftRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        leftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        lift.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        tilt.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        //Wait for the game to start
        waitForStart();

        //Autonomous Code
        driveForward(.15, 6000);
        turnLeft(.75, 550);
        driveForward(.15, 750);
        liftUp(.75, 9000);
        tiltDown(1, 400);
    }
    public void driveForward(double power, long time) throws InterruptedException{
        rightRear.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        Thread.sleep(time);
        rightRear.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
    }
    public void turnLeft(double power, long time) throws InterruptedException{
        rightRear.setPower(power);
        leftRear.setPower(-power);
        rightFront.setPower(power);
        leftFront.setPower(-power);
        Thread.sleep(time);
        rightRear.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
    }
    public void turnRight(double power, long time) throws InterruptedException{
        rightRear.setPower(-power);
        leftRear.setPower(power);
        rightFront.setPower(-power);
        leftFront.setPower(power);
        Thread.sleep(time);
        rightRear.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
    }
    public void liftUp(double power, long time) throws InterruptedException{
        lift.setPower(power);
        Thread.sleep(time);
        lift.setPower(0);
    }
    public void tiltDown(double power, long time) throws InterruptedException{
        tilt.setPower(power);
        Thread.sleep(time);
        tilt.setPower(0);
    }
    public void driveBack(double power, long time) throws InterruptedException{
        rightRear.setPower(-power);
        leftRear.setPower(-power);
        rightFront.setPower(-power);
        leftFront.setPower(-power);
        Thread.sleep(time);
        rightRear.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
    }



}
