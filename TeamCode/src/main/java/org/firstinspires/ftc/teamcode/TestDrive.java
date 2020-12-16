package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TestDrive", group="Test")
public class TestDrive extends LinearOpMode {

    float speedDivisor = 1;
    private static final float DEAD_WHEEL_LATERAL_DISTANCE = 9.3f; // in Inches
    private static final float DEAD_WHEEL_OFFSET  = 4.65f;
    private static final float WHEEL_CIRCUMFERENCE_MM = (float) (Math.PI * 50f);
    private static final float COUNTS_PER_MM = (8192f/WHEEL_CIRCUMFERENCE_MM);
    private static final float COUNTS_PER_ROTATION = 8192f;
    private static final float wheelRadius = 0.98f;
    private int leftEncoderPos = 0;
    private int centerEncoderPos = 0;
    private int rightEncoderPos = 0;
    private int leftEncoderPosPrev = 0;
    private int centerEncoderPosPrev = 0;
    private int rightEncoderPosPrev = 0;
    private double deltaLeftDistance = 0;
    private double deltaRightDistance = 0;
    private double deltaCenterDistance = 0;
    private double x = 0;
    private double y = 0;
    private double theta = 0;

    // Wheels
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor rearRightMotor = null;
    // Encoders
    //public DcMotorEx encoderLeft = null;
    //public DcMotorEx encoderRight = null;
    //public DcMotorEx encoderRear = null;

    private ElapsedTime period = new ElapsedTime();
    
// Encoder functions
     public void resetTicks() {
        resetLeftTicks();
        resetCenterTicks();
        resetRightTicks();
    }
    public void resetLeftTicks() {leftEncoderPosPrev = leftEncoderPos;}
    // arbitrary assignment of rear left motor for the center encoder, can and should be changed todo
    public int getLeftTicks() {
         leftEncoderPos = rearLeftMotor.getCurrentPosition();
         telemetry.addData("left encoder position:", leftEncoderPos);
         return leftEncoderPos - leftEncoderPosPrev;
     }

    public void resetRightTicks() {rightEncoderPosPrev = rightEncoderPos;}
    //  arbitrary assignment of rear right motor for the center encoder, can and should be changed todo
    public int getRightTicks() {
        rightEncoderPos = rearRightMotor.getCurrentPosition();
        telemetry.addData("right encoder position:", rightEncoderPos);
        return rightEncoderPos - rightEncoderPosPrev;
    }

    public void resetCenterTicks() {centerEncoderPosPrev = centerEncoderPos;}

    // arbitrary assignment of front left motor for the center encoder, can and should be changed todo
    public int getCenterTicks() {
       centerEncoderPos = frontLeftMotor.getCurrentPosition();
        telemetry.addData("center encoder position:", centerEncoderPos);
        return centerEncoderPos - centerEncoderPosPrev;
    }

    public void updatePosition() {
        deltaLeftDistance = (getLeftTicks() / COUNTS_PER_ROTATION) * 2.0 * Math.PI * wheelRadius;
        deltaRightDistance = (getRightTicks() / COUNTS_PER_ROTATION) * 2.0 * Math.PI * wheelRadius;
        deltaCenterDistance = (getCenterTicks() / COUNTS_PER_ROTATION) * 2.0 * Math.PI * wheelRadius;
        theta  += (deltaLeftDistance - deltaRightDistance) / DEAD_WHEEL_LATERAL_DISTANCE;
        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta) - deltaCenterDistance * Math.sin(theta);
        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta) + deltaCenterDistance * Math.cos(theta);

        telemetry.addData("theta:", "%.3f", theta);
        telemetry.addData("x position", "%.3f", x);
        telemetry.addData("y position", "%.3f", y);

        resetTicks();
    }

    public TestDrive() {
    }

    @Override
    public void runOpMode() {
        String result = "";
        // encoderLeft = hardwareMap.get(DcMotorEx.class, "encoder_left");
        // encoderRight = hardwareMap.get(DcMotorEx.class, "encoder_right");
        // encoderRear = hardwareMap.get(DcMotorEx.class, "encoder_rear");

        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rear_left");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rear_right");

        // encoderLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        // encoderRight.setDirection(DcMotorSimple.Direction.FORWARD);
        // encoderRear.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);


        waitForStart();

        while (opModeIsActive()) {
            
            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;
            boolean gamepad1A = gamepad1.a;
            boolean gamepad1B = gamepad1.b;
            boolean gamepad1X = gamepad1.x;

            if (gamepad1X) {
                speedDivisor = 4;
            }
            if (gamepad1A) {
                speedDivisor = 2;
            }
            if (gamepad1B) {
                speedDivisor = 1;
            }

            float frontLeftPower = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
            float frontRightPower = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
            float rearLeftPower = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
            float rearRightPower = -gamepad1LeftY - gamepad1LeftX + gamepad1RightX;


            frontLeftPower = Range.clip(frontLeftPower/speedDivisor, -1, 1);
            frontRightPower = Range.clip(frontRightPower/speedDivisor, -1, 1);
            rearLeftPower = Range.clip(rearLeftPower/speedDivisor, -1, 1);
            rearRightPower = Range.clip(rearRightPower/speedDivisor, -1, 1);

            // write the values to the motors
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            rearLeftMotor.setPower(rearLeftPower);
            rearRightMotor.setPower(rearRightPower);

            updatePosition();


            telemetry.update();
            sleep(50);
        }
    }
}