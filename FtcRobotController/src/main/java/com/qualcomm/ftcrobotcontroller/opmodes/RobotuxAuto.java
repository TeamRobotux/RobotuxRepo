package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * A simple example of a linear op mode that will approach an IR beacon
 */
public class RobotuxAuto extends LinearOpMode {

    final static double TRAY_MIN_RANGE  = 1.0;
    final static double TRAY_MAX_RANGE  = 0.1;

    final static boolean THRESHOLD_ENABLED = false;

    DcMotor rightRear;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor leftFront;
   // DcMotor lift;
    //DcMotor tilt;
    DcMotor pickUp;
    Servo lever1;
    Servo lever2;
    Servo tray;

    /*private class LimitedMotor extends DcMotor {

        public int minpos;
        public int maxpos;

        public LimitedMotor(DcMotorController controller, int portNumber) {
            super(controller, portNumber);
            minpos = Integer.MIN_VALUE;
            maxpos = Integer.MAX_VALUE;
        }

        public LimitedMotor(DcMotorController controller, int portNumber, Direction direction) {
            super(controller, portNumber, direction);
            minpos = Integer.MIN_VALUE;
            maxpos = Integer.MAX_VALUE;
        }

        public LimitedMotor(DcMotor motor, int minPosition, int maxPosition) {
            super(motor.getController(), motor.getPortNumber(), motor.getDirection());
            minpos = minPosition;
            maxpos = maxPosition;
        }

        @Override
        public void setPower(double power) {
            if (!THRESHOLD_ENABLED) {
                super.setPower(power);
                return;
            }

            if (super.getCurrentPosition() < minpos || (power > 0 && super.getCurrentPosition() < maxpos)) {
                super.setPower(power);
            } else if (super.getCurrentPosition() > maxpos || (power < 0 && super.getCurrentPosition() > minpos)) {
                super.setPower(power);
            } else {
                super.setPower(0.0);
            }
        }
    }*/
    public void driveForward (double power){
        rightRear.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        leftFront.setPower(power);
    }/*
    public void turnLeft (double power){
       // rightRear.setPower(power);
        rightFront.setPower(power);
       // leftRear.setPower(-power);
        leftFront.setPower(-power);
    }
    public void turnRight (double power){
        rightRear.setPower(-power);
        rightFront.setPower(-power);
       // leftRear.setPower(power);
       // leftFront.setPower(power) ;
    }*/
    public void driveForwardDistance (double power, int distance){
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

        while(rightFront.getCurrentPosition()!=distance){

        }
        stopDriving();
        rightRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }/*
    public void turnLeftDistance (double power, int distance){
        rightRear.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        leftRear.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        leftFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        rightFront.setTargetPosition(distance);
        rightRear.setTargetPosition(distance);
        leftFront.setTargetPosition(-distance);
        leftRear.setTargetPosition(-distance);

        rightRear.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        turnLeft(power);

        while(rightFront.isBusy()&&rightRear.isBusy()&&leftRear.isBusy()&&leftFront.isBusy()){

        }
    }
    public void turnRightDistance (double power, int distance){
        rightRear.setMode(DcMotorController.RunMode.RESET_ENCODERS);
     //   leftRear.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
     //   leftFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        rightFront.setTargetPosition(-distance);
        rightRear.setTargetPosition(-distance);
       // leftFront.setTargetPosition(distance);
        //leftRear.setTargetPosition(distance);

        rightRear.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //leftRear.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //leftFront.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        turnRight(power);

        while(rightFront.isBusy()&&rightRear.isBusy()&&leftRear.isBusy()&&leftFront.isBusy()){

        }
    }*/
    public void stopDriving(){
        driveForward(0);
    }
    @Override
    public void runOpMode() throws InterruptedException {

        // set up the hardware devices we are going to use
        rightRear = hardwareMap.dcMotor.get("motor_2b");
        leftRear = hardwareMap.dcMotor.get("motor_1b");
        rightFront = hardwareMap.dcMotor.get("motor_2f");
        leftFront = hardwareMap.dcMotor.get("motor_1f");
        pickUp = hardwareMap.dcMotor.get("flipper");
       // lift = new LimitedMotor(hardwareMap.dcMotor.get("motorLift"), 0, 10324);
       // tilt = new LimitedMotor(hardwareMap.dcMotor.get("motorTilt"), 0, 4290);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        //mode
        rightRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //lift.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
       // tilt.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //servos

        lever1 = hardwareMap.servo.get("motor_leftL");
        lever2 = hardwareMap.servo.get("motor_rightL");
        tray = hardwareMap.servo.get("motor_tray");

        // wait for the start button to be pressed
        waitForStart();
        driveForwardDistance(1,50000);

    }
}
