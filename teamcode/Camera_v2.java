package org.firstinspires.ftc.teamcode;

import android.provider.ContactsContract;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "A:Camera_v2 - detects left or right or center")
public class Camera_v2 extends LinearOpMode {
    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

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

        telemetry.addData("Detecting Location: ", detector.getLocation());
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            int i = 0;
            while (opModeIsActive()) {
                // Put loop blocks here.
                i++;
                telemetry.addData("Inside the while opModeIsActive: ", i);
                //grayPercentLeft, grayPercentCenter, cyanPercentLeft, cyanPercentCenter, magentaPercentLeft, magentaPercentCenter;
                telemetry.addData("Left Non Zeros (gray, cyan, megenta)",  "( " + Double.toString(detector.grayPercentLeft)
                                                                                       + ", " + Double.toString(detector.cyanPercentLeft)
                                                                                       + ", " + Double.toString(detector.magentaPercentLeft) + " )");
                telemetry.addData("Center Non Zeros (gray, cyan, megenta)",  "( " + Double.toString(detector.grayPercentCenter)
                                                                                        + ", " + Double.toString(detector.cyanPercentCenter)
                                                                                        + ", " + Double.toString(detector.magentaPercentCenter) + " )");
                telemetry.addData("maxPercentLeft: ", detector.getmaxPercentLeft());
                telemetry.addData("maxPercentLeft: ", detector.getmaxPercentLeft());
                telemetry.addData("maxPercentCenter: ", detector.getmaxPercentCenter());


                telemetry.addData("Detecting Location: ", detector.getLocation());
                telemetry.update();
                Utilities.Sleep(1000);
                /*try {
                    Thread.sleep(1000);
                } catch (Exception ex) {
                    telemetry.addData("Exception occurred from sleep block: ", i);
                    telemetry.update();
                }*/
            }
        }
    }
}
