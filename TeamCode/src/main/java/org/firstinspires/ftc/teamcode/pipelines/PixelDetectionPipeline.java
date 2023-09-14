package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelDetectionPipeline extends OpenCvPipeline {
    private Mat grayMat = new Mat();

    // Custom object to represent a detected hexagon
    public static class Pixel {
        public Rect boundingRect; // Rectangle bounding the detected hexagon
        public Point[] vertices; // Vertices of the hexagon
        public Point center; // Center of the hexagon

        public Pixel(Rect boundingRect, Point[] vertices, Point center) {
            this.boundingRect = boundingRect;
            this.vertices = vertices;
            this.center = center;
        }
    }

    private List<Pixel> detectedPixels = new ArrayList<>(); // List of detected hexagons

    @Override
    public Mat processFrame(Mat input) {
        // Clone the input frame to keep it intact
        Mat output = input.clone();

        // Convert the input frame to grayscale for edge detection
        Imgproc.cvtColor(input, grayMat, Imgproc.COLOR_RGB2GRAY);

        // Apply Canny edge detection
        Mat edges = new Mat();
        Imgproc.Canny(grayMat, edges, 50, 150);

        // Find contours in the edge-detected frame
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter and find hexagons among the contours
        detectedPixels.clear();
        for (MatOfPoint contour : contours) {
            MatOfPoint2f approx = new MatOfPoint2f();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double contourArea = Imgproc.contourArea(contour);

            // Approximate the contour with a polygon (hexagon)
            Imgproc.approxPolyDP(contour2f, approx, 0.03 * Imgproc.arcLength(contour2f, true), true);

            if (approx.total() == 6 && contourArea > 1000 && isHexagon(approx.toList())) {
                Point[] vertices = approx.toArray();
                Rect boundingRect = Imgproc.boundingRect(contour);

                // Calculate the center of the hexagon
                double centerX = (vertices[0].x + vertices[3].x) / 2;
                double centerY = (vertices[0].y + vertices[3].y) / 2;
                Point center = new Point(centerX, centerY);

                // Draw the hexagon and its center
                for (int j = 0; j < 6; j++) {
                    Imgproc.line(output, vertices[j], vertices[(j + 1) % 6], new Scalar(0, 255, 0), 2);
                }
                Imgproc.drawMarker(output, center, new Scalar(255, 0, 0), Imgproc.MARKER_CROSS, 20, 2);

                detectedPixels.add(new Pixel(boundingRect, vertices, center));
            }
        }

        // Return the processed frame with drawn hexagons and centers
        return output;
    }

    // Function to check if a polygon with six vertices is a hexagon
    private boolean isHexagon(List<Point> polygon) {
        if (polygon.size() != 6) {
            return false;
        }

        double[] angles = new double[6];
        for (int i = 0; i < 6; i++) {
            Point p1 = polygon.get(i);
            Point p2 = polygon.get((i + 1) % 6);
            Point p3 = polygon.get((i + 2) % 6);

            double angle = Math.toDegrees(Math.atan2(p2.y - p1.y, p2.x - p1.x) -
                    Math.atan2(p3.y - p1.y, p3.x - p1.x));

            if (angle < 0) {
                angle += 360;
            }

            angles[i] = angle;
        }

        // Check if all angles are approximately 120 degrees (hexagon)
        for (double angle : angles) {
            if (Math.abs(angle - 30) > 200) {
                return false;
            }
        }

        return true;
    }

    // Function to return the list of detected hexagons
    public List<Pixel> getDetectedPixels() {
        return detectedPixels;
    }
}
