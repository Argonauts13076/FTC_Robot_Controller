package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

@Autonomous(name = "UltimateAuto")
public class UltimateGoalAuto extends LinearOpMode {

    float speedDivisor = 1;
    float mmPerInch = 25.4f;
    float mmBotWidth = 18 * mmPerInch;
    float mmImageHeight = 6.375f * mmPerInch;
    float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;
    private static final float DEAD_WHEEL_LATERAL_DISTANCE = 14.5f; // Diameter in Inches
    private static final float DEAD_WHEEL_OFFSET = 8.5f;
    private static final float WHEEL_CIRCUMFERENCE_MM = (float) (Math.PI * 50f);
    private static final float COUNTS_PER_MM = (8192f / WHEEL_CIRCUMFERENCE_MM);
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
    private double theta = Math.PI / 2;
    int linePos = 85000;
    boolean firsttime = true;

    private int left = 0;
    private int right = 0;
    private int center = 0;
    private double rightacc = 0;
    private double leftacc = 0;
    double speed = -0.3;
    double leftoff = 0.0;
    double rightoff = 0.1;
    double fudge = -0.0;

    int intakePower, launchPower;
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

    public void resetLeftTicks() {
        leftEncoderPosPrev = leftEncoderPos;
    }

    // arbitrary assignment of rear left motor for the center encoder, can and should be changed todo
    public int getLeftTicks() {
        //facing the other direction so we must make it negative
        leftEncoderPos = -rearLeftMotor.getCurrentPosition();
        telemetry.addData("left encoder position:", leftEncoderPos);
        return leftEncoderPos - leftEncoderPosPrev;
    }

    public void resetRightTicks() {
        rightEncoderPosPrev = rightEncoderPos;
    }

    public int getRightTicks() {
        rightEncoderPos = rearRightMotor.getCurrentPosition();
        telemetry.addData("right encoder position:", rightEncoderPos);
        return rightEncoderPos - rightEncoderPosPrev;
    }

    public void resetCenterTicks() {
        centerEncoderPosPrev = centerEncoderPos;
    }

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
            c -= (r - l) / 2;

            deltaLeftDistance = (l / COUNTS_PER_ROTATION) * 2.0 * Math.PI * wheelRadius;
            deltaRightDistance = (r / COUNTS_PER_ROTATION) * 2.0 * Math.PI * wheelRadius;
            deltaCenterDistance = (c / COUNTS_PER_ROTATION) * 2.0 * Math.PI * wheelRadius;

            telemetry.addData("L,R,C", "%d,%d,%d", l, r, c);

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
        deltaTheta = (deltaLeftDistance - deltaRightDistance) / DEAD_WHEEL_LATERAL_DISTANCE;
        theta -= deltaTheta;
        telemetry.addData("center deltas", "%.2f, %.2f", deltaCenterDistance, (deltaTheta * DEAD_WHEEL_OFFSET));
        deltaCenterDistance += deltaTheta * DEAD_WHEEL_OFFSET;
        x += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta) - deltaCenterDistance * Math.sin(theta);
        y += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta) + deltaCenterDistance * Math.cos(theta);


        telemetry.addData("theta:", "%.3f", theta);
        telemetry.addData("x position", "%.3f", x);
        telemetry.addData("y position", "%.3f", y);
        telemetry.addData("deltas", "%.2f, %.2f, %.2f", deltaLeftDistance, deltaRightDistance, deltaCenterDistance);
        resetTicks();
    }

    enum AutoState {
        START,
        DRIVE,
        DETECT,
        MOVE,
        SHOOT,
        FINISH,
    }

    @Override
    public void runOpMode() {
        String result = "";
        int leftPos, rightPos;

        leftPos = rightPos = 0;

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
                .translation(-mmFTCFieldWidth / 4, mmFTCFieldWidth / 2, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));
        BlueTowerGoal.setLocationFtcFieldFromTarget(BlueTowerGoalLocationOnField);

        OpenGLMatrix RedTowerGoalLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(mmFTCFieldWidth / 4, mmFTCFieldWidth / 2, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));
        RedTowerGoal.setLocationFtcFieldFromTarget(RedTowerGoalLocationOnField);

        OpenGLMatrix FrontWallLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(0, -mmFTCFieldWidth / 2, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 180));
        FrontWall.setLocationFtcFieldFromTarget(FrontWallLocationOnField);

        OpenGLMatrix BlueAllianceLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth / 2, 0, mmImageHeight)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90));
        BlueAlliance.setLocationFtcFieldFromTarget(BlueAllianceLocationOnField);

        OpenGLMatrix RedAllianceLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(mmFTCFieldWidth / 2, 0, mmImageHeight)
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
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
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

            if (firsttime) {
                do {
                    frontRightMotor.setPower(-speed + rightoff);
                    rearLeftMotor.setPower(speed + leftoff);
                    rearRightMotor.setPower(-speed + rightoff);
                    frontLeftMotor.setPower(speed + leftoff);


                    leftPos = rearLeftMotor.getCurrentPosition();
                    rightPos = rearRightMotor.getCurrentPosition();

                    telemetry.addData("ls,rs: ", "%.2f, %.2f", speed + leftoff, -speed + rightoff);
                    telemetry.addData("l,r", "%d, %d", leftPos, rightPos);
                    telemetry.update();

                    if ((leftPos + rightPos) > 500) {
                        leftoff += 0.005;
                    }

                    if ((leftPos + rightPos) < -500) {
                        rightoff -= 0.005;
                    }

                    if (rightoff <= -0.2) {
                        rightoff = -0.2;
                    }

                    if (leftoff >= 0.2) {
                        leftoff = 0.2;
                    }
//                if ((rightoff <= -0.2) || (leftoff >= 0.2)) {
                    //                  leftoff -= 0.05;
                    //                rightoff += 0.05;
                    //          }
                    opModeIsActive();
                    sleep(100);
                } while (leftPos < linePos && rightPos > -linePos);


                launchMotor.setPower(1.0);

                frontRightMotor.setPower(0);
                rearLeftMotor.setPower(0);
                rearRightMotor.setPower(0);
                frontLeftMotor.setPower(0);

                sleep(3000);
                opModeIsActive();
                actuator.setPosition(0.4);
                sleep(500);
                actuator.setPosition(0.0);
                sleep(500);

                actuator.setPosition(0.4);
                sleep(500);
                actuator.setPosition(0.0);
                sleep(500);
                opModeIsActive();
                intakeMotor.setPower(-0.6);
                sleep(2000);
                intakeMotor.setPower(0.0);
                opModeIsActive();
                actuator.setPosition(0.4);
                sleep(500);
                actuator.setPosition(0.0);
                sleep(500);
                opModeIsActive();
                launchMotor.setPower(0.0);
                linePos += 10000;
                do {
                    frontRightMotor.setPower(-speed + rightoff);
                    rearLeftMotor.setPower(speed + leftoff);
                    rearRightMotor.setPower(-speed + rightoff);
                    frontLeftMotor.setPower(speed + leftoff);


                    leftPos = rearLeftMotor.getCurrentPosition();
                    rightPos = rearRightMotor.getCurrentPosition();

                    telemetry.addData("ls,rs: ", "%.2f, %.2f", speed + leftoff, -speed + rightoff);
                    telemetry.addData("l,r", "%d, %d", leftPos, rightPos);
                    telemetry.update();

                    if ((leftPos + rightPos) > 500) {
                        leftoff += 0.01;
                    }

                    if ((leftPos + rightPos) < -500) {
                        rightoff -= 0.01;
                    }

                    if (rightoff <= -0.2) {
                        rightoff = -0.2;
                    }

                    if (leftoff >= 0.2) {
                        leftoff = 0.2;
                    }
//                if ((rightoff <= -0.2) || (leftoff >= 0.2)) {
                    //                  leftoff -= 0.05;
                    //                rightoff += 0.05;
                    //          }
                    opModeIsActive();
                    sleep(50);
                } while (leftPos < linePos && rightPos > -linePos);

                frontRightMotor.setPower(0);
                rearLeftMotor.setPower(0);
                rearRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                sleep(50);
            }
            firsttime = false;
        }
    }
}

