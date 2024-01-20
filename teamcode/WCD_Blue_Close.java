package org.firstinspires.ftc.teamcode;

//THIS ONE WORKS

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "WCD_Blue_Close")
public class WCD_Blue_Close extends LinearOpMode {
    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    double MoveEncoderPosition = 0;
    double CenterSpikeDropOff = 1550;
    int CenterYellowDropTurn = -800;
    int CenterSpikeDropOff1 = 1000;
    double BlueCloseLeft = -450;
    double BlueCloseRightTurn = 1050;
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
        // Put initialization blocks here.
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",
                hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);

        CenterStageDetection detector = new CenterStageDetection();
        webcam1.setPipeline(detector);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        CenterStageDetection.ColorDetected colorLeft = detector.getColorLeft();
        CenterStageDetection.ColorDetected colorMiddle = detector.getColorMiddle();
        while(!isStopRequested() && !isStarted()) {
            telemetry.addData("Detecting Location: ", detector.getLocation());
            telemetry.update();
        }



        waitForStart();
        if (opModeIsActive()) {

            if (detector.getLocation() == org.firstinspires.ftc.teamcode.CenterStageDetection.Location.CENTER) {
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
                BL.setPower(-0.35);
                FR.setPower(-0.35);
                BR.setPower(-0.35);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition >= CenterSpikeDropOff - 100)) {
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

                while (!(isStopRequested() || MoveEncoderPosition <= -50)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }


                FL.setPower(0.5);
                BL.setPower(-0.5);
                FR.setPower(-0.5);
                BR.setPower(0.5);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition <= -1300)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }

                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                FL.setPower(-0.5);
                BL.setPower(-0.5);
                FR.setPower(-0.5);
                BR.setPower(-0.5);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition >= 1050)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }

                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                FL.setPower(0.5);
                BL.setPower(0.5);
                FR.setPower(-0.5);
                BR.setPower(-0.5);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition <= CenterYellowDropTurn)) {
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

                FL.setPower(-0.5);
                BL.setPower(-0.5);
                FR.setPower(-0.5);
                BR.setPower(-0.5);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition >= 250)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

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

                FL.setPower(0.25);
                BL.setPower(0.25);
                FR.setPower(0.25);
                BR.setPower(0.25);
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

                FL.setPower(0.5);
                BL.setPower(-0.5);
                FR.setPower(-0.5);
                BR.setPower(0.5);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition <= -3000)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }

                FL.setPower(-0.35);
                BL.setPower(-0.35);
                FR.setPower(-0.35);
                BR.setPower(-0.35);
                sleep(3500);


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

            }
            if (detector.getLocation() == org.firstinspires.ftc.teamcode.CenterStageDetection.Location.LEFT) {
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

                FL.setPower(0.35);
                BL.setPower(-0.35);
                FR.setPower(-0.35);
                BR.setPower(0.35);

                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition <= BlueCloseLeft)) {
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

                while (!(isStopRequested() || MoveEncoderPosition >= CenterSpikeDropOff1 + 100)) {
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

                while (!(isStopRequested() || MoveEncoderPosition <= -50)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }


                FL.setPower(0.5);
                BL.setPower(-0.5);
                FR.setPower(-0.5);
                BR.setPower(0.5);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition <= -900)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }

                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                FL.setPower(-0.5);
                BL.setPower(-0.5);
                FR.setPower(-0.5);
                BR.setPower(-0.5);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition >= 900)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }

                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                FL.setPower(0.5);
                BL.setPower(0.5);
                FR.setPower(-0.5);
                BR.setPower(-0.5);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition <= CenterYellowDropTurn)) {
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

                FL.setPower(-0.5);
                BL.setPower(-0.5);
                FR.setPower(-0.5);
                BR.setPower(-0.5);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition >= 275)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

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

                FL.setPower(0.25);
                BL.setPower(0.25);
                FR.setPower(0.25);
                BR.setPower(0.25);
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

                FL.setPower(0.5);
                BL.setPower(-0.5);
                FR.setPower(-0.5);
                BR.setPower(0.5);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition <= -3000)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }

                FL.setPower(-0.35);
                BL.setPower(-0.35);
                FR.setPower(-0.35);
                BR.setPower(-0.35);
                sleep(3500);


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

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                sleep(30000);

                        }
            if(detector.getLocation() == org.firstinspires.ftc.teamcode.CenterStageDetection.Location.RIGHT){
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

                FL.setPower(0.45);
                BL.setPower(-0.45);
                FR.setPower(-0.45);
                BR.setPower(0.45);

                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition <= BlueCloseLeft)) {
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

                FL.setPower(-0.45);
                BL.setPower(-0.45);
                FR.setPower(-0.45);
                BR.setPower(-0.45);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition >= CenterSpikeDropOff1 + 300)) {
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

                FL.setPower(-0.45);
                BL.setPower(-0.45);
                FR.setPower(0.45);
                BR.setPower(0.45);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition >= BlueCloseRightTurn)) {
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
                while (!(isStopRequested() || MoveEncoderPosition >= CenterSpikeDropOff1 -100)) {
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

                FL.setPower(0.45);
                BL.setPower(0.45);
                FR.setPower(0.45);
                BR.setPower(0.45);
                MoveEncoderPosition = BR.getCurrentPosition();
                while (!(isStopRequested() || MoveEncoderPosition <= -1000)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                sleep(100);

                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                FL.setPower(0.45);
                BL.setPower(0.45);
                FR.setPower(-0.45);
                BR.setPower(-0.45);
                MoveEncoderPosition = BR.getCurrentPosition();
                while (!(isStopRequested() || MoveEncoderPosition <= -BlueCloseRightTurn)) {
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

                FL.setPower(0.5);
                BL.setPower(0.5);
                FR.setPower(-0.5);
                BR.setPower(-0.5);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition <= CenterYellowDropTurn)) {
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

                FL.setPower(-0.5);
                BL.setPower(-0.5);
                FR.setPower(-0.5);
                BR.setPower(-0.5);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition >= 775)) {
                    MoveEncoderPosition = BR.getCurrentPosition();
                    telemetry.addData("Movement Encoder Postion", MoveEncoderPosition);
                    telemetry.update();
                    sleep(20);
                }

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

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
                sleep(1500);

                FL.setPower(0.25);
                BL.setPower(0.25);
                FR.setPower(0.25);
                BR.setPower(0.25);
                MoveEncoderPosition = BR.getCurrentPosition();

                while (!(isStopRequested() || MoveEncoderPosition <= -25)) {
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

                FL.setPower(0.45);
                BL.setPower(-0.45);
                FR.setPower(-0.45);
                BR.setPower(0.45);
                sleep(2000);

                FL.setPower(-0.45);
                BL.setPower(-0.45);
                FR.setPower(-0.45);
                BR.setPower(-0.45);
                sleep(1900);


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

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                sleep(30000);
            }
        }

    }
}



