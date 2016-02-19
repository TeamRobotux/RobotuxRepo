package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*11/12/15*/
public class RobotuxDrive2015_16 extends OpMode {

    DcMotor rightRear;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor lift;
    DcMotor tilt;
    DcMotor pickUp;
    Servo leftLever;
    Servo rightLever;
    Servo tray;

    public void start() {

    }

    public void init() {
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

    }
    public void loop(){
        float leftMotors = gamepad1.left_stick_y;
        float rightMotors = gamepad1.left_stick_y;

      /*  boolean turn = gamepad1.left_stick_x;
        if(turn){
            leftMotors = gamepad1.left_stick_x;
            rightMotors = -gamepad1.right_stick_x;
        }



        leftMotors = Range.clip(leftMotors, -1, 1);
        rightMotors = Range.clip(rightMotors, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        leftMotors = (float)scaleInput(leftMotors);
        rightMotors =  (float)scaleInput(rightMotors);

        rightRear.setPower(rightMotors);
        rightFront.setPower(rightMotors);
        leftRear.setPower(leftMotors);
        leftFront.setPower(leftMotors);*/
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
    public void stop (){

    }
}
