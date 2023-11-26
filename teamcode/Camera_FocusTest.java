package org.firstinspires.ftc.teamcode;

import android.provider.ContactsContract;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utilities;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "A:Camera Focus Test -  left and center")
public class Camera_FocusTest extends LinearOpMode {
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
        webcam1.setPipeline( new examplePipeLine());
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            int i = 0;
            while (opModeIsActive()) {
                // Put loop blocks here.
                i++;
                telemetry.addData("Inside the while opModeIsActive: ", i);
                telemetry.update();
                Utilities.Sleep(1000);
                /*try {
                    Thread.sleep(1000);
                } catch (Exception ex) {
                    telemetry.addData("Exception occurred from sleep block: ", i);
                    telemetry.update();
                } */
            }
        }
    }
    class examplePipeLine extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar redColor = new Scalar(255.0,0.0,0.0);
        Scalar greenColor = new Scalar(0.0,255.0,0.0);
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("Pipleline Running... !");

            // 320 x 240
            Rect leftRect = new Rect(12,50,90,120);
            Rect rightRect = new Rect(125,40,180,50);

            //Rect leftRect = new Rect(new Point(10,110), new Point(70,220));
            //Rect rightRect = new Rect(new Point(135,130), new Point(215,320));

            input.copyTo(outPut);
            Imgproc.rectangle(outPut,leftRect,redColor,2);
            Imgproc.rectangle(outPut,rightRect,greenColor,2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop,leftCrop,2);
            Core.extractChannel(rightCrop,rightCrop,2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];

            if(leftavgfin > rightavgfin) {
                telemetry.addLine("Left");
            }
            else {
                telemetry.addLine("Right");
            }
            return (outPut);
        }
    }
}
