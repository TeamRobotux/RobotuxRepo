package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class RobotuxAuto2 extends LinearOpMode {

    DcMotor rightRear;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor leftFront;

    public void driveForward (double power)throws InterruptedException {
        rightRear.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        leftFront.setPower(power);
    }
    public void driveForwardDistance (double power, int distance) throws InterruptedException {
        rightRear.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        leftRear.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        leftFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        rightFront.setTargetPosition(distance);
        rightRear.setTargetPosition(distance);
        leftFront.setTargetPosition(distance);
        leftRear.setTargetPosition(distance);

        rightRear.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        driveForward(power);
        telemetry.addData("1right front: ", rightFront.getCurrentPosition());
        telemetry.addData("1right back: ", rightRear.getCurrentPosition());
        while(rightFront.isBusy()){
            telemetry.addData("2right front: ", rightFront.getCurrentPosition());
            telemetry.addData("2right back: ", rightRear.getCurrentPosition());
        }
        telemetry.addData("3right front: ", rightFront.getCurrentPosition());
        telemetry.addData("3right back: ", rightRear.getCurrentPosition());
        stopDriving();
        telemetry.addData("text: ", "hello2");
        rightRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    }
    public void stopDriving(){
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
    }

    public void runOpMode() throws InterruptedException {

        // set up the hardware devices we are going to use
        rightRear = hardwareMap.dcMotor.get("motor_2b");
        leftRear = hardwareMap.dcMotor.get("motor_1b");
        rightFront = hardwareMap.dcMotor.get("motor_2f");
        leftFront = hardwareMap.dcMotor.get("motor_1f");
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);


        //mode
        rightRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        // wait for the start button to be pressed
        waitForStart();
        telemetry.addData("text: ", "hello1");
        driveForwardDistance(1.0, 1000);
        telemetry.addData("text: ", "hello3");


    }
}
