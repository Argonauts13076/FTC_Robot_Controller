package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Test: Basic Drive Train", group="Test")
public class BasicDriveTrainTeleOp extends LinearOpMode {

    float speedDivisor = 1;
    double scissorLiftSpeed = 1;
    boolean scissorLiftLatch = true;
    boolean gripperLatch = true;

    BasicDriveTrainHardware hardware = new BasicDriveTrainHardware();


    @Override
    public void runOpMode() {
        String s = hardware.init(hardwareMap);

        scissorLiftSpeed = BasicDriveTrainHardware.ScissorLiftPower;

        waitForStart();

        //hardware.OpenGripper();

        while (opModeIsActive()) {

            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;
            boolean gamepad1A = gamepad1.a;
            boolean gamepad1B = gamepad1.b;
            boolean gamepad1X = gamepad1.x;

            boolean gamepad2DPadUp = gamepad2.dpad_up;
            boolean gamepad2DPadDown = gamepad2.dpad_down;

// Aidan driving practice
//            boolean gamepad1DPadUp = gamepad1.dpad_up;
//            boolean gamepad1DPadDown = gamepad1.dpad_down;
//            boolean gamepad1Y = gamepad1.y;


            float gamepad2LeftTrigger = gamepad2.left_trigger;
            float gamepad2RightTrigger = gamepad2.right_trigger;
            boolean gamepad2A = gamepad2.a;
            boolean gamepad2B = gamepad2.b;
            boolean gamepad2X = gamepad2.x;

            if(hardware.canUseWheels) {
                if (gamepad1X) {
                    speedDivisor = 4;
                }
                if (gamepad1A) {
                    speedDivisor = 2;
                }
                if (gamepad1B) {
                    speedDivisor = 1;
                }

                float FrontLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
                float FrontRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
                float BackLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
                float BackRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;

                // clip the right/left values so that the values never exceed +/- 1
                FrontLeft = Range.clip(FrontLeft, -1, 1);
                FrontRight = Range.clip(FrontRight, -1, 1);
                BackLeft = Range.clip(BackLeft, -1, 1);
                BackRight = Range.clip(BackRight, -1, 1);

                // write the values to the motors
                hardware.FrontLeft.setPower(-FrontLeft / speedDivisor);
                hardware.FrontRight.setPower(-FrontRight / speedDivisor);
                hardware.RearLeft.setPower(-BackLeft / speedDivisor);
                hardware.RearRight.setPower(-BackRight / speedDivisor);
            }
        }
    }
}

          /*  if(hardware.canUseScissorLift) {
                if (gamepad2LeftTrigger > 0.5 && gamepad2RightTrigger > 0.5) {
                    hardware.scissorLiftManualOverride();
                } else {
                    hardware.revokeScissorLiftManualOverride();
                }

                if (gamepad2X) {
                    scissorLiftSpeed = BasicDriveTrainHardware.ScissorLiftPartialPower;
                }
                if (gamepad2B) {
                    scissorLiftSpeed = BasicDriveTrainHardware.ScissorLiftPower;
                }

                if (hardware.getManualOverride()) {
                    if (!(gamepad2DPadUp == gamepad2DPadDown)) {
                        if (gamepad2DPadUp) {
                            hardware.ScissorLift.setPower(-scissorLiftSpeed);
                        }
                        if (gamepad2DPadDown) {
                            hardware.ScissorLift.setPower(scissorLiftSpeed);
                        }
                    } else {
                        hardware.ScissorLift.setPower(0);
                    }
                } else {
                    if (!(gamepad2DPadUp == gamepad2DPadDown)) {
                        if (scissorLiftLatch) {
                            if (gamepad2DPadUp) {
                                hardware.incrementPosition();
                                scissorLiftLatch = false;
                            }
                            if (gamepad2DPadDown) {
                                hardware.decrementPosition();
                                scissorLiftLatch = false;
                            }
                        }
                        /*
                    } else if (!(gamepad1DPadUp == gamepad1DPadDown)) {
                        if (scissorLiftLatch) {
                            if (gamepad1DPadUp) {
                                hardware.incrementPosition();
                                scissorLiftLatch = false;
                            }
                            if (gamepad1DPadDown) {
                                hardware.decrementPosition();
                                scissorLiftLatch = false;
                            }
                        }
                         */
            /*
                    } else {
                        scissorLiftLatch = true;

                    }
                }
            }
 */

/*
            if(hardware.canUseGripper) {
                if (gamepad2A) {
//                if (gamepad2A || gamepad1Y) {
                    if (gripperLatch) {
                        gripperLatch = false;
                        if (hardware.getGripperState()) {
                            hardware.CloseGripper();
                        } else {
                            hardware.OpenGripper();
                        }
                    }
                } else {
                    gripperLatch = true;
                }
            }
            telemetry.addData("scissorPos", hardware.getScissorEncoderCount() );
            telemetry.update();
            sleep(50);
        }
    }
}

*/