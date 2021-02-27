package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="UltimateGoalTeleOp")
public class UltimateGoalTeleOp extends LinearOpMode {

    float speedDivisor = 1;
    float mmPerInch = 25.4f;
    float mmBotWidth = 18 * mmPerInch;
    float mmImageHeight = 6.375f * mmPerInch;
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;
    private static final float DEAD_WHEEL_LATERAL_DISTANCE = 14.5f; // Diameter in Inches
    private static final float DEAD_WHEEL_OFFSET  = 8.5f;
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
    private double theta = Math.PI/2;

    private int left = 0;
    private int right = 0;
    private int center = 0;
    private double rightacc = 0;
    private double leftacc = 0;

    double intakePower, launchPower;
    int intakeState = 0;
    //Vuforia
    VuforiaLocalizer vuforia;
    WebcamName webcamName;
    OpenGLMatrix lastLocation = null;

    // Wheels
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor rearLeftMotor = null;
    public DcMotor rearRightMotor = null;
    public DcMotor intakeMotor = null;
    public DcMotor launchMotor = null;
    public Servo actuator = null;
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
         //facing the other direction so we must make it negative
         leftEncoderPos = -rearLeftMotor.getCurrentPosition();
         telemetry.addData("left encoder position:", leftEncoderPos);
         return leftEncoderPos - leftEncoderPosPrev;
     }

    public void resetRightTicks() {rightEncoderPosPrev = rightEncoderPos;}

    public int getRightTicks() {
        rightEncoderPos = rearRightMotor.getCurrentPosition();
        telemetry.addData("right encoder position:", rightEncoderPos);
        return rightEncoderPos - rightEncoderPosPrev;
    }

    public void resetCenterTicks() {centerEncoderPosPrev = centerEncoderPos;}

    public int getCenterTicks() {
       centerEncoderPos = frontLeftMotor.getCurrentPosition();
        telemetry.addData("center encoder position:", centerEncoderPos);
        return centerEncoderPos - centerEncoderPosPrev;
    }

    public void updatePosition() {
        double deltaTheta;
    if (true) {
        int l, r, c;

        l = getLeftTicks();
        r = getRightTicks();
        c = getCenterTicks();
        c -= (r - l)/2;

        deltaLeftDistance = (l / COUNTS_PER_ROTATION) * 2.0 * Math.PI * wheelRadius;
        deltaRightDistance = (r / COUNTS_PER_ROTATION) * 2.0 * Math.PI * wheelRadius;
        deltaCenterDistance = (c / COUNTS_PER_ROTATION) * 2.0 * Math.PI * wheelRadius;

        telemetry.addData("L,R,C", "%d,%d,%d", l,r,c);

    } else {
        left = 10; // * (int) Math.round(9 + Math.random());
        right = -10; //* (int) Math.round(9 + Math.random());
        center = 0;

        deltaLeftDistance = (left / COUNTS_PER_ROTATION) * 2.0 * Math.PI * wheelRadius;
        deltaRightDistance = (right / COUNTS_PER_ROTATION) * 2.0 * Math.PI * wheelRadius;
        deltaCenterDistance = (center / COUNTS_PER_ROTATION) * 2.0 * Math.PI * wheelRadius;
        rightacc += deltaRightDistance;
        leftacc += deltaLeftDistance;

        telemetry.addData("L,R", "%d, %d", left, right);
        telemetry.addData("LAcc, RAcc", "%.2f, %.2f", leftacc, rightacc);

    }
        deltaTheta  = (deltaLeftDistance - deltaRightDistance) / DEAD_WHEEL_LATERAL_DISTANCE;
        theta -= deltaTheta;
        telemetry.addData("center deltas", "%.2f, %.2f", deltaCenterDistance, (deltaTheta * DEAD_WHEEL_OFFSET));
        deltaCenterDistance += deltaTheta * DEAD_WHEEL_OFFSET;
        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta) - deltaCenterDistance * Math.sin(theta);
        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta) + deltaCenterDistance * Math.cos(theta);


        telemetry.addData("theta:", "%.3f", theta);
        telemetry.addData("x position", "%.3f", x);
        telemetry.addData("y position", "%.3f", y);
        telemetry.addData("deltas", "%.2f, %.2f, %.2f", deltaLeftDistance, deltaRightDistance, deltaCenterDistance);
        resetTicks();
    }

    public UltimateGoalTeleOp() {
    }

    @Override
    public void runOpMode() {
        String result = "";
        boolean actuatorRest = true;

        webcamName = hardwareMap.get(WebcamName.class, "webcam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AdlTbyT/////AAABmRzwXqB2QkgjoGVrNLf2cn1fVKNUyrJY52lWZSj6LLjLBbboeY6C+yHcl6Z2IktdtHNUkPoYp4NCpGu5Gi4WJZ+LYITD/iHVbH+AcfUijOwAtAlyczB7zogzmQ4cTwL8f71iwtt6tn1zb9hhFL32l3nEmeqw4wE0j/c5Cbw8oObewGJkPNBtcVNdsu8fGw8MxCYgXL+JgvTvY9UXtcl4vt9dlW/wwGo5oScv5iuH3gFVQfQSg88YdT7VEuPGmY1eXMEwlpLllzvpCNueMnR7ZzbZHJS6/JGPIrCwiAzTOQUTtha8/9doDiR7wPfKD6h0+WQSh8nbPFlxETcIm9h+DM6Fq7/0tIaN2cRkSqPGRFyy";
        parameters.cameraName = webcamName;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToBitmap();


        VuforiaTrackables UGTrackables = vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable BlueTowerGoal = UGTrackables.get(0);
        BlueTowerGoal.setName("BlueTowerGoal");

        VuforiaTrackable RedTowerGoal = UGTrackables.get(1);
        RedTowerGoal.setName("RedTowerGoal");

        VuforiaTrackable RedAlliance = UGTrackables.get(2);
        RedAlliance.setName("RedAlliance");

        VuforiaTrackable BlueAlliance = UGTrackables.get(3);
        BlueAlliance.setName("BlueAlliance");

        VuforiaTrackable FrontWall = UGTrackables.get(4);
        FrontWall.setName("FrontWall");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(UGTrackables);

        OpenGLMatrix BlueTowerGoalLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/4, mmFTCFieldWidth/2, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));
        BlueTowerGoal.setLocationFtcFieldFromTarget(BlueTowerGoalLocationOnField);

        OpenGLMatrix RedTowerGoalLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(mmFTCFieldWidth/4, mmFTCFieldWidth/2, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));
        RedTowerGoal.setLocationFtcFieldFromTarget(RedTowerGoalLocationOnField);

        OpenGLMatrix FrontWallLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(0, -mmFTCFieldWidth/2, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 180));
        FrontWall.setLocationFtcFieldFromTarget(FrontWallLocationOnField);

        OpenGLMatrix BlueAllianceLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 0, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90));
        BlueAlliance.setLocationFtcFieldFromTarget(BlueAllianceLocationOnField);

        OpenGLMatrix RedAllianceLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(mmFTCFieldWidth/2, 0, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 90));
        RedAlliance.setLocationFtcFieldFromTarget(RedAllianceLocationOnField);
        OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(0, 0, 0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ,
                AngleUnit.DEGREES, 0, 0, 0));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        }

        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "rear_left");
        rearRightMotor = hardwareMap.get(DcMotor.class, "rear_right");
        launchMotor = hardwareMap.get(DcMotor.class, "launch_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        actuator = hardwareMap.get(Servo.class, "actuator");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        actuator.setDirection(Servo.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        launchMotor.setPower(0);
        intakeMotor.setPower(0);
        actuator.setPosition(0.0);
        waitForStart();

        while (opModeIsActive()) {
            boolean gamepad1DPadUp = gamepad1.dpad_up;
            boolean gamepad1DPadDown = gamepad1.dpad_down;
            boolean gamepad1LeftBumper = gamepad1.left_bumper;
            boolean gamepad1Rightbumper = gamepad1.right_bumper;
            float gamepad1LeftY = gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;
            boolean gamepad1A = gamepad1.a;
            boolean gamepad1B = gamepad1.b;
            boolean gamepad1X = gamepad1.x;
            boolean gamepad1Y = gamepad1.y;
            if (gamepad1X) {
                speedDivisor = 4;
            }
            if (gamepad1A) {
                speedDivisor = 2;
            }
            if (gamepad1B) {
                speedDivisor = 1;
            }
            if (gamepad1DPadUp) {
                launchPower = -1;
            }
            if (gamepad1DPadDown) {
                launchPower = 0;
            }
            if (gamepad1Rightbumper) {
                if (intakeState < 1) {
                    intakeState++;
                }
            }
            if (gamepad1LeftBumper) {
                if (intakeState > -1) {
                    intakeState--;
                }
            }
            if (gamepad1LeftBumper || gamepad1Rightbumper) {

                if (intakeState == 1) {
                    intakePower = -0.6;
                } else if (intakeState == -1) {
                    intakePower = 0.6;
                } else {
                    intakePower = 0;
                }
                sleep(100);
            }


            if (gamepad1Y) {
                 if (actuatorRest == true) {
                     actuator.setPosition(0.4);
                     actuatorRest = false;
                 } else {
                     actuator.setPosition(0.0);
                     actuatorRest = true;
                 }
                 sleep(100);
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
            launchMotor.setPower(launchPower);
            intakeMotor.setPower(intakePower);
            updatePosition();

            UGTrackables.activate();
             for (VuforiaTrackable trackable : allTrackables) {
                 if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                     telemetry.addData("found", trackable.getName());

                     OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();

                     if (robotLocationTransform != null) {
                         lastLocation = robotLocationTransform;
                     }
                 }
            }
            if (lastLocation != null) {
                telemetry.addData("Pos", lastLocation.formatAsTransform());
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
            sleep(50);
        }
    }
}