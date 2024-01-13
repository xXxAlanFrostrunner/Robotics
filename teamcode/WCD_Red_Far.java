package org.firstinspires.ftc.teamcode;

//THIS ONE WORKS

import android.graphics.Picture;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.hardware.Servo;



@Autonomous(name = "WCD_Red_Far")
public class WCD_Red_Far extends LinearOpMode {
    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    double MoveEncoderPosition = 0;
    double CenterSpikeDropOff = 1450;
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
    private Servo PixelHolder;

    @Override
    public void runOpMode() {
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        TC = hardwareMap.get(DcMotor.class, "TC");

        PixelHolder = hardwareMap.get(Servo.class, "PixelHolder");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        YawPitchRollAngles orientation;

        imu = hardwareMap.get(IMU.class, "imu");

        double ServoPosition;

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // Put initialization blocks here.
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        CenterStageDetection detector = new CenterStageDetection();
        webcam1.setPipeline(detector);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        CenterStageDetection.ColorDetected colorLeft = detector.getColorLeft();
        CenterStageDetection.ColorDetected colorMiddle = detector.getColorMiddle();
        while (!isStopRequested() && !isStarted()) {
            if(gamepad1.dpad_down) {
                timer -= 500;
                sleep(350);
            }
            if(gamepad1.dpad_up){
                timer += 500;
                sleep(350);

                if(timer > 15000)
                    timer = 15000;
            }

            telemetry.addData("Timer",timer);
            telemetry.addData("Detecting Location: ", detector.getLocation());
            telemetry.update();
        }

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.

            if (detector.getLocation() == org.firstinspires.ftc.teamcode.CenterStageDetection.Location.CENTER) {
                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                // Reset the motor encoder

                FL.setPower(-0.35);
                BL.setPower(-0.35);
                FR.setPower(-0.35);
                BR.setPower(-0.35);
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

                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                FL.setPower(0.35);
                BL.setPower(0.35);
                FR.setPower(0.35);
                BR.setPower(0.35);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition <= -1265)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                sleep(timer + 6000);
                FL.setPower(-0.5);
                BL.setPower(0.5);
                FR.setPower(0.5);
                BR.setPower(-0.5);
                sleep(5500);

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
            }
            if (detector.getLocation() == org.firstinspires.ftc.teamcode.CenterStageDetection.Location.RIGHT) {
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
                while (!(isStopRequested() || MoveEncoderPosition >= 1145)) {
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
            }
            if (detector.getLocation() == org.firstinspires.ftc.teamcode.CenterStageDetection.Location.LEFT) {
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

                FL.setPower(0.35);
                BL.setPower(0.35);
                FR.setPower(0.35);
                BR.setPower(0.35);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition <= -1100)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

//                sleep(1800);
//
//                FL.setPower(-0.35);
//                BL.setPower(-0.35);
//                FR.setPower(-0.35);
//                BR.setPower(-0.35);
//                sleep(125);

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                sleep(timer + 4000);

                FL.setPower(-0.5);
                BL.setPower(0.5);
                FR.setPower(0.5);
                BR.setPower(-0.5);
                sleep(7000);


            }
        }
    }


}




