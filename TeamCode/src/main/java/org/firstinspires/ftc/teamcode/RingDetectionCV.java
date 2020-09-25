package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;

@Autonomous
public class RingDetectionCV extends LinearOpMode {
    OpenCvCamera phoneCam;
    StageSwitchingPipeline pipeline;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        pipeline = new StageSwitchingPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
                telemetry.addData("pos: ", pipeline.getAnalysis());
                telemetry.addData("thresh: ", pipeline.getThreshold());
                telemetry.update();
            }
    }

    public static class StageSwitchingPipeline extends OpenCvPipeline {
        enum ringPos
        {
            Four,
            One,
            None
        }
        static final Point topLeft = new Point(40,160);
        static final int width = 25;
        static final int height = 35;
        final int fourRingsThreshold = 100;
        final int oneRingThreshold = 122;
        Point pointA = new Point(topLeft.x, topLeft.y);
        Point pointB = new Point(topLeft.x + width, topLeft.y + height);

        Mat box = new Mat();
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avgCb;

        private volatile ringPos pos = ringPos.Four;

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
            box = Cb.submat(new Rect(pointA, pointB));
        }
        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avgCb = (int) Core.mean(box).val[0];
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    pointA, // First point which defines the rectangle
                    pointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), 1);
            if(avgCb < oneRingThreshold)
            {
                pos = ringPos.One;
            }
            if(avgCb < fourRingsThreshold) {
                pos = ringPos.Four;
            }
            if(avgCb > 122)
            {
                pos = ringPos.None;
            }
            return input;
        }
        public ringPos getAnalysis()
        {
            return pos;
        }
        public int getThreshold()
        {
            return avgCb;
        }
    }
}