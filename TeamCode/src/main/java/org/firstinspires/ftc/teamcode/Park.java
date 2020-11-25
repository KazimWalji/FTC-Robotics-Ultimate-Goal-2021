
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

@Autonomous
public class Park extends LinearOpMode
{
    OpenCvCamera phoneCam;
    StageSwitchingPipeline pipeline;

    BNO055IMU imu;

    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lr = null;
    private DcMotor rr = null;
    private Servo wobble = null;
    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        pipeline = new StageSwitchingPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        lf = hardwareMap.get(DcMotor.class, "left_front");
        rf = hardwareMap.get(DcMotor.class, "right_front");
        lr = hardwareMap.get(DcMotor.class, "left_rear");
        rr = hardwareMap.get(DcMotor.class, "right_rear");
        wobble = hardwareMap.get(Servo.class, "scoop_servo");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        double threshold = pipeline.getThreshold();
        RingDetectionCV.StageSwitchingPipeline.ringPos pos = pipeline.getAnalysis();

        drive.setPoseEstimate(new Pose2d(-63,  -53, 0));
        Pose2d startpos = new Pose2d(-63,  -53, 0);


        while (opModeIsActive()) {
            wobble.setPosition(0.2);
                Trajectory traj = drive.trajectoryBuilder(startpos)
                        .lineTo(new Vector2d(30, -45))
                        .build();

                drive.followTrajectory(traj);

            Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                    .back(20)
                    .build();

            drive.followTrajectory(traj1);

                sleep(300000);

        }
    }
    public static class StageSwitchingPipeline extends OpenCvPipeline {
        enum ringPos
        {
            Four,
            One,
            None
        }
        static final Point topLeft = new Point(40,180);
        static final int width = 25;
        static final int height = 35;
        final int fourRingsThreshold = 110;
        final int oneRingThreshold = 122;
        Point pointA = new Point(topLeft.x, topLeft.y);
        Point pointB = new Point(topLeft.x + width, topLeft.y + height);

        Mat box = new Mat();
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avgCb;

        private volatile RingDetectionCV.StageSwitchingPipeline.ringPos pos = RingDetectionCV.StageSwitchingPipeline.ringPos.Four;

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
                pos = RingDetectionCV.StageSwitchingPipeline.ringPos.One;
            }
            if(avgCb < fourRingsThreshold) {
                pos = RingDetectionCV.StageSwitchingPipeline.ringPos.Four;
            }
            if(avgCb > 122)
            {
                pos = RingDetectionCV.StageSwitchingPipeline.ringPos.None;
            }
            return input;
        }
        public RingDetectionCV.StageSwitchingPipeline.ringPos getAnalysis()
        {
            return pos;
        }
        public int getThreshold()
        {
            return avgCb;
        }
    }
}
