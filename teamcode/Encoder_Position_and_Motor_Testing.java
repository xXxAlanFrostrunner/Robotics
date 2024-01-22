package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Encoder_Positon+Motor Testing")
public class Encoder_Position_and_Motor_Testing extends LinearOpMode {

    int MoveEncoderPosition = 0;
    int CenterSpikeDropOff = 1550;
    int CenterYellowDropTurn = -800;

    int CenterSpikeDropOff1 = 1000;
    double RedFarLeft = -400;
    double RedFarRightTurn = 1100;
    double BlueCloseLeft = -450;
    double BlueCloseRightTurn = 1050;
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
    private TouchSensor TopTouchSensor;
    private TouchSensor BottomTouchSensor;
    private Servo GreenGrip;

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

        TopTouchSensor = hardwareMap.get(TouchSensor.class, "6");
        BottomTouchSensor = hardwareMap.get(TouchSensor.class, "mjolnere");

        GreenGrip = hardwareMap.get(Servo.class, "GreenGrip");

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
        GreenGrip.setPosition(0.234);
        waitForStart();
        if (opModeIsActive()) {


        }
        GreenGrip.setPosition(0.234);

        TC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Reset the motor encoder
        GreenGrip.setPosition(0.234);

        FL.setPower(-0.35);
        BL.setPower(0.35);
        FR.setPower(0.35);
        BR.setPower(-0.35);
        MoveEncoderPosition = BR.getCurrentPosition();

        while (!(isStopRequested() || MoveEncoderPosition >= 100)) {
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
        BL.setPower(-0.35);
        FR.setPower(-0.35);
        BR.setPower(-0.35);
        MoveEncoderPosition = BR.getCurrentPosition();

        while (!(isStopRequested() || MoveEncoderPosition >= CenterSpikeDropOff+100)) {
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

        FL.setPower(0.35);
        BL.setPower(0.35);
        FR.setPower(0.35);
        BR.setPower(0.35);
        MoveEncoderPosition = BR.getCurrentPosition();

        while (!(isStopRequested() || MoveEncoderPosition <= -300)) {
            MoveEncoderPosition = BR.getCurrentPosition();
            telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
            telemetry.update();
            sleep(20);
        }

        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(-0.5);
        BL.setPower(-0.5);
        FR.setPower(0.5);
        BR.setPower(0.5);
        MoveEncoderPosition = BR.getCurrentPosition();

        while (!(isStopRequested() || MoveEncoderPosition >= 1000)) {
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
        FR.setPower(-0.5);
        BR.setPower(-0.5);
        MoveEncoderPosition = BR.getCurrentPosition();

        while (!(isStopRequested() || MoveEncoderPosition >= 1450)) {
            MoveEncoderPosition = BR.getCurrentPosition();
            telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
            telemetry.update();
            sleep(20);
        }

        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(0.35);
        BL.setPower(-0.35);
        FR.setPower(-0.35);
        BR.setPower(0.35);
        MoveEncoderPosition = BR.getCurrentPosition();

        while (!(isStopRequested() || MoveEncoderPosition <= -180)) {
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

        TR.setPower(1);
        TL.setPower(1);
        MoveEncoderPosition = TR.getCurrentPosition();

        while(!(isStopRequested() || MoveEncoderPosition >= 3500)) {
            MoveEncoderPosition = TR.getCurrentPosition();
            telemetry.addData("Scissor Lift Encoder Position", MoveEncoderPosition);
            telemetry.update();
            sleep(20);
        }

        TR.setPower(0);
        TL.setPower(0);

        TC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        FL.setPower(-0.5);
//        BL.setPower(-0.5);
//        FR.setPower(-0.5);
//        BR.setPower(-0.5);
//        MoveEncoderPosition = BR.getCurrentPosition();
//
//        while (!(isStopRequested() || MoveEncoderPosition >= 125)) {
//            MoveEncoderPosition = BR.getCurrentPosition();
//            telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
//            telemetry.update();
//            sleep(20);
//        }
//
//        FL.setPower(0);
//        BL.setPower(0);
//        FR.setPower(0);
//        BR.setPower(0);

        TC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TC.setPower(-0.45);
        MoveEncoderPosition = TC.getCurrentPosition();

        while (!(isStopRequested() || MoveEncoderPosition <= -250)) {
            MoveEncoderPosition = TC.getCurrentPosition();
            telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
            telemetry.update();
            sleep(20);
        }
        TC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GreenGrip.setPosition(0.466);

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        TC.setPower(0);
        sleep(3000);

        FL.setPower(0.5);
        BL.setPower(0.5);
        FR.setPower(0.5);
        BR.setPower(0.5);
        MoveEncoderPosition = BR.getCurrentPosition();

        while (!(isStopRequested() || MoveEncoderPosition <= -275)) {
            MoveEncoderPosition = BR.getCurrentPosition();
            telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
            telemetry.update();
            sleep(20);
        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        TR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TR.setPower(-1);
        TL.setPower(-1);
        MoveEncoderPosition = TR.getCurrentPosition();

        while(!(isStopRequested() || MoveEncoderPosition <= -3000)) {
            MoveEncoderPosition = TR.getCurrentPosition();
            telemetry.addData("Scissor Lift Encoder Position", MoveEncoderPosition);
            telemetry.update();
            sleep(20);
        }

        TR.setPower(0);
        TL.setPower(0);

        GreenGrip.setPosition(0.234);

        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setPower(-0.5);
        BL.setPower(0.5);
        FR.setPower(0.5);
        BR.setPower(-0.5);
        MoveEncoderPosition = BR.getCurrentPosition();

        while (!(isStopRequested() || MoveEncoderPosition >= 2300)) {
            MoveEncoderPosition = BR.getCurrentPosition();
            telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
            telemetry.update();
            sleep(20);
        }

        FL.setPower(-0.35);
        BL.setPower(-0.35);
        FR.setPower(-0.35);
        BR.setPower(-0.35);
        sleep(2500);


        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        sleep(30000);

        TC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        TC.setPower(0);


        while (opModeIsActive()) {

//                    if (BottomTouchSensor.isPressed()) {
//                        TC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        TC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    }
//                    if (TopTouchSensor.isPressed()) {
//                        TC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        TC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    }
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

            TR.setPower(gamepad1.right_stick_x);
            TL.setPower(gamepad1.right_stick_x);

            TC.setPower(gamepad1.right_stick_y);

            orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("BR current position: ", BR.getCurrentPosition());
            telemetry.addData("TC current position: ", TC.getCurrentPosition());
            telemetry.addData("TR current position: ", TR.getCurrentPosition());
//                            telemetry.addData("Yaw (Z)", JavaUtil.formatNumber(orientation.getYaw(AngleUnit.DEGREES), 2));
            telemetry.update();
            if (gamepad1.a) {
                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                TC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                TC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

        }

        telemetry.update();
    }

}

