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

    // Wheels
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor rearRightMotor = null;

    public DcMotorEx encoderLeft = null;
    public DcMotorEx encoderRight = null;
    public DcMotorEx encoderRear = null;

    private ElapsedTime period = new ElapsedTime();

    public TestDrive() {
    }

    @Override
    public void runOpMode() {
        String result = "";
            encoderLeft = hardwareMap.get(DcMotorEx.class, "encoder_left");
            encoderRight = hardwareMap.get(DcMotorEx.class, "encoder_right");
            encoderRear = hardwareMap.get(DcMotorEx.class, "encoder_rear");

            frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
            frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
            rearLeftMotor = hardwareMap.get(DcMotor.class, "rear_left");
            rearRightMotor = hardwareMap.get(DcMotor.class, "rear_right");

            encoderLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            encoderRight.setDirection(DcMotorSimple.Direction.FORWARD);
            encoderRear.setDirection(DcMotorSimple.Direction.FORWARD);

            frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

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

            float frontLeftPower = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float frontRightPower = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float rearLeftPower = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float rearRightPower = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

            frontLeftPower = Range.clip(frontLeftPower, -1, 1);
            frontRightPower = Range.clip(frontRightPower, -1, 1);
            rearLeftPower = Range.clip(rearLeftPower, -1, 1);
            rearRightPower = Range.clip(rearRightPower, -1, 1);

            // write the values to the motors
            frontLeftMotor.setPower(frontLeftPower / speedDivisor);
            frontRightMotor.setPower(frontRightPower / speedDivisor);
            rearLeftMotor.setPower(rearLeftPower / speedDivisor);
            rearRightMotor.setPower(rearRightPower / speedDivisor);

            telemetry.update();
            sleep(50);
        }
}


}