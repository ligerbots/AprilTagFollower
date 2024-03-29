// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tagfinder;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterPivot;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * This is a demo program showing the detection of AprilTags. The image is acquired from the USB
 * camera, then any detected AprilTags are marked up on the image and sent to the dashboard.
 *
 * <p>Be aware that the performance on this is much worse than a coprocessor solution!
 */
public class AprilTagFinder extends Command {
    private final DriveTrain m_driveTrain;
    private final ShooterPivot m_shooterPivot;
    private final PositionCorrection m_positionCorrection;

    AprilTagDetector detector = new AprilTagDetector();
    AprilTagPoseEstimator.Config poseEstConfig;
    AprilTagPoseEstimator estimator;
    UsbCamera camera;
    CvSink cvSink;
    CvSource outputStream;
    Mat mat, grayMat;
    ArrayList<Long> tags;
    Scalar outlineColor, crossColor;
    NetworkTable tagsTable;
    IntegerArrayPublisher pubTags;

    Transform3d currentPosition;
    private void setCurrentPosition(Transform3d currentPosition) {
        this.currentPosition = currentPosition;
    }
    public Transform3d getCurrentPosition() {
        return currentPosition;
    }

    public AprilTagFinder(DriveTrain driveTrain, ShooterPivot shooterPivot) {
        super();
        m_driveTrain = driveTrain;
        m_shooterPivot = shooterPivot;
        m_positionCorrection = new PositionCorrection(m_driveTrain, m_shooterPivot, this::getCurrentPosition);

        detector = new AprilTagDetector();
        // look for tag36h11, correct 3 error bits
        detector.addFamily("tag36h11", 3);

        // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
        // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
        poseEstConfig = new AprilTagPoseEstimator.Config(
            0.1651,
            699.3778103158814, 677.7161226393544,
            345.6059345433618, 207.12741326228522
        );
        estimator = new AprilTagPoseEstimator(poseEstConfig);

        // Get the UsbCamera from CameraServer
        camera = CameraServer.startAutomaticCapture();
        // Set the resolution
        camera.setResolution(640, 480);

        // Get a CvSink. This will capture Mats from the camera
        cvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        outputStream = CameraServer.putVideo("Detected", 640, 480);


        // Mats are very memory expensive. Lets reuse these.
        mat = new Mat();
        grayMat = new Mat();

        // Instantiate once
        tags = new ArrayList<>();
        outlineColor = new Scalar(0, 255, 0);
        crossColor = new Scalar(0, 0, 255);

        // We'll output to NT
        tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
        pubTags = tagsTable.getIntegerArrayTopic("tags").publish();
        m_positionCorrection.deadlineWith(this).schedule();
    }

    void findAprilTagAndMove() {
        // Tell the CvSink to grab a frame from the camera and put it
        // in the source mat.  If there is an error notify the output.
        if (cvSink.grabFrame(mat) == 0) {
            // Send the output the error.
            outputStream.notifyError(cvSink.getError());
            // skip the rest of the current iteration
            return;
        }

        Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

        AprilTagDetection[] detections = detector.detect(grayMat);

        // have not seen any tags yet
        tags.clear();

        for (AprilTagDetection detection : detections) {
            // remember we saw this tag
            tags.add((long) detection.getId());

            // draw lines around the tag
            for (int i = 0; i <= 3; i++) {
                int j = (i + 1) % 4;
                Point pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
                Point pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
                Imgproc.line(mat, pt1, pt2, outlineColor, 2);
            }

            // mark the center of the tag
            double cx = detection.getCenterX();
            double cy = detection.getCenterY();
            int ll = 10;
            Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), crossColor, 2);
            Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), crossColor, 2);

            // identify the tag
            Imgproc.putText(
                mat,
                Integer.toString(detection.getId()),
                new Point(cx + ll, cy),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                1,
                crossColor,
                3);

            // determine pose
            Transform3d pose = estimator.estimate(detection);

            // put pose into dashboard
            Rotation3d rot = pose.getRotation();
            tagsTable.getEntry("pose_" + detection.getId()).setDoubleArray(new double[] {
                pose.getX(), pose.getY(), pose.getZ(),
                rot.getX(), rot.getY(), rot.getZ()
            });
            SmartDashboard.putString(
                "apriltag/transform",
                "(%.2f %.2f $.2f) / (%.2f %.2f %.2f)".formatted(
                    pose.getX(), pose.getY(), pose.getZ(),
                    rot.getX(), rot.getY(), rot.getZ()
                )
            );

            // follow apriltag
            tags.stream()
                .filter(l -> l.longValue() == 1)
                .findFirst()
                .ifPresent(l -> setCurrentPosition(pose));
        }

        // put list of tags onto dashboard
        pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());

        // Give the output stream a new image to display
        outputStream.putFrame(mat);
    }

    @Override
    public void initialize() {
        
    }
    @Override
    public void execute() {
        findAprilTagAndMove();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        pubTags.close();
        detector.close();
    }
} 