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

@TeleOp(name = "A:Camera_v3 - detect left or right or top")
public class Camera_v3 extends LinearOpMode {
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
                // possible 640x360; 640x480; 544x288; 752x416;800x448;800x600;960x544;960x720;1024x576;1184x656;1280x720
                webcam1.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
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
                try {
                    Thread.sleep(1000);
                } catch (Exception ex) {
                    telemetry.addData("Exception occurred from sleep block: ", i);
                    telemetry.update();
                }
            }
        }
    }
    class examplePipeLine extends OpenCvPipeline{
        Mat matYCrCb = new Mat();
        Mat leftCrop;
        Mat centerCrop;
        Mat rightCrop;
        double leftavgfin;
        double centeravgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);
        Rect leftRect = new Rect(1,1,219,959);
        Rect centerRect = new Rect(221,1,839,959);
        Rect rightRect = new Rect(1061,1,219,959);


        public Mat processFrame(Mat input){
            telemetry.addLine("Pipleline Running... !");
            Imgproc.cvtColor(input,matYCrCb,Imgproc.COLOR_RGB2YCrCb);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut,leftRect,rectColor,2);
            Imgproc.rectangle(outPut,centerRect,rectColor,2);
            Imgproc.rectangle(outPut,rightRect,rectColor,2);

            leftCrop = matYCrCb.submat(leftRect);
            rightCrop = matYCrCb.submat(rightRect);
            centerCrop = matYCrCb.submat(centerRect);

            Core.extractChannel(leftCrop,leftCrop,2);
            Core.extractChannel(centerCrop,centerCrop,2);
            Core.extractChannel(rightCrop,rightCrop,2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar centeravg = Core.mean(centerCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            centeravgfin = rightavg.val[0];
            rightavgfin = centeravg.val[0];
            telemetry.addData("Left average - ",leftavgfin);
            telemetry.addData("Center average - ",centeravgfin);
            telemetry.addData("Right average - ",rightavgfin);
            if(leftavgfin > rightavgfin) {
                telemetry.addLine("Left");
            }
            else {
                telemetry.addLine("Right");
            }

            try {
                Thread.sleep(1000);
            } catch (Exception ex) {
                telemetry.addLine("Exception occurred from sleep block: ");
                telemetry.update();
            }
            return (outPut);
        }
    }
}
