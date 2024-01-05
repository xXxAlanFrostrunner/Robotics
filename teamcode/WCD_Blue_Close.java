package org.firstinspires.ftc.teamcode;

//THIS ONE WORKS

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

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
                webcam1.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
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

                while (!(isStopRequested() || MoveEncoderPosition >= CenterSpikeDropOff-100)) {
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
                sleep(1937);

                FL.setPower(0.5);
                BL.setPower(-0.5);
                FR.setPower(-0.5);
                BR.setPower(0.5);
                sleep(2500);

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                sleep(30000);
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
                sleep(2500);

                FL.setPower(0.35);
                BL.setPower(-0.35);
                FR.setPower(-0.35);
                BR.setPower(0.35);
                sleep(3500);
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);

                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                sleep(30000);

                        }
            if(detector.getLocation() == org.firstinspires.ftc.teamcode.CenterStageDetection.Location.RIGHT){
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

                while (!(isStopRequested() || MoveEncoderPosition >= CenterSpikeDropOff1 + 400)) {
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

                FL.setPower(0.35);
                BL.setPower(0.35);
                FR.setPower(0.35);
                BR.setPower(0.35);
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
                sleep(3000);

//            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                FL.setPower(-0.35);
                BL.setPower(0.35);
                FR.setPower(0.35);
                BR.setPower(-0.35);
                MoveEncoderPosition = BR.getCurrentPosition();
                while (!(isStopRequested() || MoveEncoderPosition >= 500)) {
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
                sleep(2000);
            }
        }

    }
}



