package org.firstinspires.ftc.teamcode.objectdetection;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.ArrayList;
import java.util.List;

public class ColorDetectionPipeline extends OpenCvPipeline {
    private Scalar lowerGreen = new Scalar(35, 50, 50);   // Lower HSV range for green
    private Scalar upperGreen = new Scalar(85, 255, 255); // Upper HSV range for green
    private Scalar lowerPurple = new Scalar(120, 50, 50); // Lower HSV range for purple
    private Scalar upperPurple = new Scalar(160, 255, 255); // Upper HSV range for purple
    private Scalar lowerYellow = new Scalar(20, 50, 50);  // Lower HSV range for yellow
    private Scalar upperYellow = new Scalar(30, 255, 255); // Upper HSV range for yellow
    private Scalar lowerWhite = new Scalar(0, 0, 200);    // Lower HSV range for white
    private Scalar upperWhite = new Scalar(180, 30, 255); // Upper HSV range for white

    // Custom object to represent a detected color region
    public static class ColorRegion {
        public Rect boundingRect; // Rectangle bounding the detected region
        public Point center; // Center of the detected region
        public Point relativePosition; // Relative position to the camera's view

        public ColorRegion(Rect boundingRect, Point center) {
            this.boundingRect = boundingRect;
            this.center = center;
        }
    }

    private List<ColorRegion> colorRegions = new ArrayList<>(); // List of detected color regions

    @Override
    public Mat processFrame(Mat input) {
        // Clone the input frame to keep it intact
        Mat output = input.clone();

        // Convert the input frame to the HSV color space
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Create masks for each color
        Mat maskGreen = new Mat();
        Mat maskPurple = new Mat();
        Mat maskYellow = new Mat();
        Mat maskWhite = new Mat();
        Core.inRange(hsv, lowerGreen, upperGreen, maskGreen);
        Core.inRange(hsv, lowerPurple, upperPurple, maskPurple);
        Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);
        Core.inRange(hsv, lowerWhite, upperWhite, maskWhite);

        // Combine masks to detect all colors
        Mat combinedMask = new Mat();
        Core.add(maskGreen, maskPurple, combinedMask);
        Core.add(combinedMask, maskYellow, combinedMask);
        Core.add(combinedMask, maskWhite, combinedMask);

        // Find contours in the combined mask
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(combinedMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw rectangles around detected regions
        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            Imgproc.rectangle(output, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2);
        }

        colorRegions = getDetectedRegions(output);
        // Return the processed frame with drawn rectangles
        return output;
    }

    // Function to return the list of detected color regions
    public List<ColorRegion> getDetectedRegions(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat maskGreen = new Mat();
        Mat maskPurple = new Mat();
        Mat maskYellow = new Mat();
        Mat maskWhite = new Mat();
        Core.inRange(hsv, lowerGreen, upperGreen, maskGreen);
        Core.inRange(hsv, lowerPurple, upperPurple, maskPurple);
        Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);
        Core.inRange(hsv, lowerWhite, upperWhite, maskWhite);

        Mat combinedMask = new Mat();
        Core.add(maskGreen, maskPurple, combinedMask);
        Core.add(combinedMask, maskYellow, combinedMask);
        Core.add(combinedMask, maskWhite, combinedMask);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(combinedMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<ColorRegion> detectedRegions = new ArrayList<>();
        Point cameraViewCenter = new Point(input.width() / 2, input.height() / 2);

        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            Point center = new Point(boundingRect.x + boundingRect.width / 2, boundingRect.y + boundingRect.height / 2);
            ColorRegion colorRegion = new ColorRegion(boundingRect, center);
            double xOffset = center.x - cameraViewCenter.x;
            double yOffset = center.y - cameraViewCenter.y;
            colorRegion.relativePosition = new Point(xOffset, yOffset);
            detectedRegions.add(colorRegion);
        }

        return detectedRegions;
    }

    public List<ColorRegion> getColorRegions() {
        return colorRegions;
    }
}