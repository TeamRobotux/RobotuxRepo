package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class RobotuxDrive3_16 extends OpMode {
    final static double LEVER1_MIN_RANGE  = 1.0;
    final static double LEVER1_MAX_RANGE  = 0.5;
    final static double LEVER2_MIN_RANGE  = 0;
    final static double LEVER2_MAX_RANGE  = 0.5;
    final static double TRAY_MIN_RANGE  = 1.0;
    final static double TRAY_MAX_RANGE  = 0.1;

    final static boolean THRESHOLD_ENABLED = false;

    DcMotor rightRear;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor lift;
    DcMotor tilt;
    DcMotor pickUp;
    Servo lever1;
    Servo lever2;
    Servo tray;


    private class LimitedMotor extends DcMotor {

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
    }
    public void start() {    }


    public void init() {
        //motors
        rightRear = hardwareMap.dcMotor.get("motor_2b");
        leftRear = hardwareMap.dcMotor.get("motor_1b");
        rightFront = hardwareMap.dcMotor.get("motor_2f");
        leftFront = hardwareMap.dcMotor.get("motor_1f");
        pickUp = hardwareMap.dcMotor.get("flipper");
        lift = new LimitedMotor(hardwareMap.dcMotor.get("motorLift"), 0, 10324); // TODO: set min and max values for tilt here
        tilt = new LimitedMotor(hardwareMap.dcMotor.get("motorTilt"), 0, 4290);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        //servos

        lever1 = hardwareMap.servo.get("motor_leftL");
        lever2 = hardwareMap.servo.get("motor_rightL");
        tray = hardwareMap.servo.get("motor_tray");

    }
    double trayPosition = 1.0;
    public void loop(){
        float leftMotors = 0;
        float rightMotors = 0;
        double lever1Position = .15;
        double lever2Position = .15;

        if(gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0){
            leftMotors = gamepad1.left_stick_y;
            rightMotors = gamepad1.left_stick_y;
        }
        if(gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0){
            leftMotors = gamepad1.right_stick_x;
            rightMotors = -gamepad1.right_stick_x;
        }

//LIFT
        if (gamepad1.y) {
            lift.setPower(1.0);
            System.out.println("robo lift: " + lift.getCurrentPosition());

        } else if (gamepad1.a) {
            lift.setPower(-1.0);
            System.out.println("robo lift: " + lift.getCurrentPosition());

        }
        else{
            lift.setPower(0);
        }
        //levers
        /*if (gamepad2.a){
            lever2 =
        }
        if (gamepad2.x){
            lever1 =
        }*/
        //pickup
        if(gamepad1.dpad_down) {
            pickUp.setPower(0.50);
        }
        if(gamepad1.dpad_up){
            pickUp.setPower(-0.50);
        }
        //tray
        if(gamepad1.dpad_left){
            trayPosition=TRAY_MAX_RANGE;
        }
        if(gamepad1.dpad_right){
            trayPosition=TRAY_MIN_RANGE;
        }

        if (gamepad2.x) {
            lever1Position = LEVER1_MIN_RANGE;
        } else {
            lever1Position = LEVER1_MAX_RANGE;
        }

        if (gamepad2.b) {
            lever2Position = LEVER2_MAX_RANGE;
        } else {
            lever2Position = LEVER2_MIN_RANGE;
        }
        //tilting
        if (gamepad1.x) {
            tilt.setPower(0.5);

            System.out.println("robo tilt: " + tilt.getCurrentPosition());

        } else if (gamepad1.b) {
            tilt.setPower(-0.5);

            System.out.println("robo tilt: " + tilt.getCurrentPosition());

        } else {
            tilt.setPower(0);
        }
        if (gamepad1.left_bumper||gamepad1.right_bumper){
            pickUp.setPower(0);
        }
        leftMotors = Range.clip(leftMotors, -1, 1);
        rightMotors = Range.clip(rightMotors, -1, 1);

        lever1Position= Range.clip(lever1Position, LEVER1_MIN_RANGE, LEVER1_MAX_RANGE);
        lever2Position = Range.clip(lever2Position, LEVER2_MIN_RANGE, LEVER2_MAX_RANGE);
        //trayPosition = Range.clip(trayPosition, TRAY_MIN_RANGE, TRAY_MAX_RANGE);

        lever1.setPosition(lever1Position);
        lever2.setPosition(lever2Position);
        tray.setPosition(trayPosition);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        leftMotors = (float)scaleInput(leftMotors);
        rightMotors =  (float)scaleInput(rightMotors);

        rightRear.setPower(rightMotors);
        rightFront.setPower(rightMotors);
        leftRear.setPower(leftMotors);
        leftFront.setPower(leftMotors);
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
