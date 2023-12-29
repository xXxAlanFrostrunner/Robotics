package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Encoder_Positon+Motor Testing")
public class AutoDrive_v1 extends LinearOpMode {

    int MoveEncoderPosition = 0;
    int CenterSpikeDropOff = 1450;

    int CenterSpikeDropOff1 = 1000;
    double RedFarLeft = -400;
    double RedFarRightTurn = 1100;
    int timer = 15000;
    private IMU imu;

    // Control Hub - motors : GoBILDA 5203
    // A - BR - port 1
    // B - BL - port 3
    // Y - FR - port 0
    // X - FL - port 2

    // Extension Hub
    // port 0 - TR
    // port 1 - TL
    // port 2 - open
    // port 3 - TC
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor TL;
    private DcMotor TR;
    private DcMotor TC;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        TC = hardwareMap.get(DcMotor.class, "TC");
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        YawPitchRollAngles orientation;
        AngularVelocity angularVelocity;

        imu = hardwareMap.get(IMU.class, "imu");

        // Initialize the IMU.
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        // Prompt user to press start button.
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {

            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Reset the motor encoder

            FL.setPower(0.35);
            BL.setPower(-0.35);
            FR.setPower(-0.35);
            BR.setPower(0.35);

            MoveEncoderPosition = BR.getCurrentPosition();

            while (!(isStopRequested() || MoveEncoderPosition <= RedFarLeft)) {
                MoveEncoderPosition = BR.getCurrentPosition();
                telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                telemetry.update();
                sleep(20);
            }
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);

            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Reset the motor encoder

            FL.setPower(-0.35);
            BL.setPower(-0.35);
            FR.setPower(-0.35);
            BR.setPower(-0.35);
            MoveEncoderPosition = BR.getCurrentPosition();

            while (!(isStopRequested() || MoveEncoderPosition >= CenterSpikeDropOff - 300)) {
                MoveEncoderPosition = BR.getCurrentPosition();
                telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                telemetry.update();
                sleep(20);
            }
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);

            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            FL.setPower(-0.5);
            BL.setPower(-0.5);
            FR.setPower(0.5);
            BR.setPower(0.5);
            MoveEncoderPosition = BR.getCurrentPosition();

            while (!(isStopRequested() || MoveEncoderPosition >= RedFarRightTurn)) {
                MoveEncoderPosition = BR.getCurrentPosition();
                telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                telemetry.update();
                sleep(20);
            }
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);

            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            FL.setPower(-0.35);
            BL.setPower(-0.35);
            FR.setPower(-0.35);
            BR.setPower(-0.35);
            MoveEncoderPosition = BR.getCurrentPosition();
            while (!(isStopRequested() || MoveEncoderPosition >= CenterSpikeDropOff1 - 150)) {
                MoveEncoderPosition = BR.getCurrentPosition();
                telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                telemetry.update();
                sleep(20);
            }
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);

            FL.setPower(0.35);
            BL.setPower(0.35);
            FR.setPower(0.35);
            BR.setPower(0.35);
            MoveEncoderPosition = BR.getCurrentPosition();
            while (!(isStopRequested() || MoveEncoderPosition <= -250)) {
                MoveEncoderPosition = BR.getCurrentPosition();
                telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                telemetry.update();
                sleep(20);
            }
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);

            FL.setPower(-0.35);
            BL.setPower(0.35);
            FR.setPower(0.35);
            BR.setPower(-0.35);
            MoveEncoderPosition = BR.getCurrentPosition();
            while (!(isStopRequested() || MoveEncoderPosition >= 1100)) {
                MoveEncoderPosition = BR.getCurrentPosition();
                telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                telemetry.update();
                sleep(20);
            }
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);

            FL.setPower(0.35);
            BL.setPower(0.35);
            FR.setPower(0.35);
            BR.setPower(0.35);
            sleep(1000);

            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            sleep(timer);

            FL.setPower(-0.45);
            BL.setPower(-0.45);
            FR.setPower(-0.45);
            BR.setPower(-0.45);
            sleep(7500);

            //            // Put run blocks here
//
//           /*
//            BL.setPower(.5);
//                BR.setPower(.5);
//                FL.setPower(.5);
//                FR.setPower(.5);
//            org.firstinspires.ftc.teamcode.Utilities.Sleep(1000);
//            BL.setPower(0);
//            BR.setPower(0);
//            FL.setPower(0);
//            FR.setPower(0);
//*/
//            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            // Reset the motor encoder
//
//            FL.setPower(-0.35);
//            BL.setPower(0.35);
//            FR.setPower(0.35);
//            BR.setPower(-0.35);

                        MoveEncoderPosition = BR.getCurrentPosition();

                        while (!(isStopRequested() || MoveEncoderPosition >= CenterSpikeDropOff)) {
                            MoveEncoderPosition = BR.getCurrentPosition();
                            telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                            telemetry.update();
                            sleep(20);
                        }
                        FL.setPower(0);
                        BL.setPower(0);
                        FR.setPower(0);
                        BR.setPower(0);

            while (opModeIsActive()) {
                BL.setPower(gamepad1.left_stick_y);
                BR.setPower(gamepad1.left_stick_y);
                FL.setPower(gamepad1.left_stick_y);
                FR.setPower(gamepad1.left_stick_y);

                BL.setPower(gamepad1.right_stick_x);
                BR.setPower(-gamepad1.right_stick_x);
                FL.setPower(gamepad1.right_stick_x);
                FR.setPower(-gamepad1.right_stick_x);

                BL.setPower(-gamepad1.right_trigger);
                BR.setPower(gamepad1.right_trigger);
                FL.setPower(gamepad1.right_trigger);
                FR.setPower(-gamepad1.right_trigger);

                BL.setPower(gamepad1.left_trigger);
                BR.setPower(-gamepad1.left_trigger);
                FL.setPower(-gamepad1.left_trigger);
                FR.setPower(gamepad1.left_trigger);


                            orientation = imu.getRobotYawPitchRollAngles();
                telemetry.addData("TR current position: ", BR.getCurrentPosition());
//                            telemetry.addData("Yaw (Z)", JavaUtil.formatNumber(orientation.getYaw(AngleUnit.DEGREES), 2));
                            telemetry.update();
                if (gamepad1.a) {
                    BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                imu.resetYaw();
                }

            }
            /*
            BL.setPower(-.5);
            BR.setPower(-.5);
            FL.setPower(-.5);
            FR.setPower(-.5);
            Utilities.Sleep(1000);

            //TL.setPower(-.5);
            //TR.setPower(-.5);
            //TC.setPower(-0.3);

            //TC.setPower(-0.2);

             */
            telemetry.update();
        }
    }
}




