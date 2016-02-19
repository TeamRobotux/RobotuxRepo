package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Julia on 1/4/2016.
 */
public class RobotuxAut1 extends LinearOpMode{

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
    public void runOpMode(){

    }
    public void main() throws InterruptedException {

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
        tray=hardwareMap.servo.get("motor_tray");
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // set the mode
        rightRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        leftRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        leftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        lift.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        tilt.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        //Wait for the game to start
       //waitForStart();

        //Autonomous Code

    }
    public void driveForward(double power) {
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightRear.setPower(power);
        leftRear.setPower(power);
    }


    public void turnLeft(double power) {
        rightFront.setPower(power);
        leftFront.setPower(-power);
        rightRear.setPower(power);
        leftRear.setPower(-power);
    }

    public void turnRight(double power){
        turnLeft(-power);
    }



}