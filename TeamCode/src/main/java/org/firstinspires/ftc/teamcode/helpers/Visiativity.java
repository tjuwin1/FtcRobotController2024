package org.firstinspires.ftc.teamcode.helpers;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.List;

public class Visiativity {
    AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder().build();
    PropFinder propFinder = new PropFinder();
    VisionPortal visionPortal;
    WebcamName[] cameraNames;

    public Visiativity(WebcamName... cvCameras){
        this.cameraNames = cvCameras;
        if(cvCameras.length > 1){
            CameraName switchableCamera = ClassFactory.getInstance()
                    .getCameraManager().nameForSwitchableCamera(cvCameras[0], cvCameras[1]);
            visionPortal = VisionPortal.easyCreateWithDefaults(switchableCamera, aprilTagProcessor, propFinder);
        }else{
            visionPortal = VisionPortal.easyCreateWithDefaults(cvCameras[0], aprilTagProcessor, propFinder);
        }
    }

    public void setStreaming(boolean startstop){
        if(startstop == false){
            visionPortal.stopStreaming();
        }else{
            visionPortal.resumeStreaming();
        }
    }

    public void switchCamera() {
        if (this.cameraNames.length > 1)
            if (visionPortal.getActiveCamera().equals(this.cameraNames[0])) {
                visionPortal.setActiveCamera(this.cameraNames[1]);
            } else {
                visionPortal.setActiveCamera(this.cameraNames[0]);
            }
    }

    public List<AprilTagDetection> getAprilTagDetections(){
        return aprilTagProcessor.getDetections();
    }

    public int getAprilTagDetectionCount(){
        return JavaUtil.listLength(aprilTagProcessor.getDetections());
    }

    public int getPropDetectionLocation(){
        return 0;
    }

    private static class PropFinder implements VisionProcessor{
        @Override
        public void init(int width, int height, CameraCalibration calibration) {}

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            /*try{
                Mat frame1, frame2;
                frame1 = new Mat();
                frame2 = new Mat();

                FinderColor = Core.mean(frame.submat(rectFinder));

                Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
                Core.inRange(frame, lower1, upper1, frame1);
                Core.inRange(frame, lower2, upper2, frame2);
                Core.bitwise_or(frame1,frame2,frame);

                satRectLeft = getAvgSaturation(frame, rectLeft);
                satRectRight = getAvgSaturation(frame, rectRight);
                satRectMiddle = getAvgSaturation(frame, rectMiddle);

                if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
                    selection = Selected.LEFT;
                } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)) {
                    selection = Selected.MIDDLE;
                }else{
                    selection = Selected.RIGHT;
                }

                return frame;
            }
            catch (Exception e){
                return frame;
            }*/
            return frame;
        }


        protected double getAvgSaturation(Mat input, Rect rect) {
            try{
                Mat submat = input.submat(rect);
                Scalar color = Core.mean(submat);
                return color.val[0];
            }
            catch (Exception e){
                return 0;
            }
        }

        private Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
            int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
            int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
            int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
            int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
            return new Rect(left, top, right, bottom);
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            /*Paint selectedPaint = new Paint();
            selectedPaint.setColor(Color.RED);
            selectedPaint.setStyle(Paint.Style.STROKE);
            selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

            Paint nonSelectedPaint = new Paint(selectedPaint);
            nonSelectedPaint.setColor(Color.GREEN);

            android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft,
                    scaleBmpPxToCanvasPx);
            android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle,
                    scaleBmpPxToCanvasPx);
            android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight,
                    scaleBmpPxToCanvasPx);

            switch (selection) {
                case LEFT:
                    canvas.drawRect(drawRectangleLeft, selectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                    break;
                case MIDDLE:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, selectedPaint);
                    canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                    break;
                case RIGHT:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    canvas.drawRect(drawRectangleRight, selectedPaint);
                    break;
                case NONE:
                    canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                    canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                    canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                    break;
            }*/
        }


/*        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // this method processes the image (frame) taken by the camera, and tries to find a suitable prop
            // you dont need to call it
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

            // thats why you need to give your scalar upper and lower bounds as HSV values
            // this method makes the colour image black and white, with everything between your upper and lower bound values as white, and everything else black
            Core.inRange(frame, lower, upper, frame);

            // this empties out the list of found contours, otherwise we would keep all the old ones, read on to find out more about contours!
            contours.clear();

            // this finds the contours, which are borders between black and white, and tries to simplify them to make nice outlines around potential objects
            // this basically helps us to find all the shapes/outlines of objects that exist within our colour range
            Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // this sets up our largest contour area to be 0
            largestContourArea = -1;
            // and our currently found largest contour to be null
            largestContour = null;

            // gets the current minimum area from min area
            double minArea = this.minArea.getAsDouble();

            // finds the largest contour!
            // for each contour we found before we loop over them, calculate their area,
            // and then if our area is larger than our minimum area, and our currently found largest area
            // it stores the contour as our largest contour and the area as our largest area
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > largestContourArea && area > minArea) {
                    largestContour = contour;
                    largestContourArea = area;
                }
            }

            // sets up the center points of our largest contour to be -1 (offscreen)
            largestContourX = largestContourY = -1;

            // if we found it, calculates the actual centers
            if (largestContour != null) {
                Moments moment = Imgproc.moments(largestContour);
                largestContourX = (moment.m10 / moment.m00);
                largestContourY = (moment.m01 / moment.m00);
            }

            // determines the current prop position, using the left and right dividers we gave earlier
            // if we didn't find any contours which were large enough, sets it to be unfound
            PropPositions propPosition;
            if (largestContour == null) {
                propPosition = PropPositions.UNFOUND;
            } else if (largestContourX < left.getAsDouble()) {
                propPosition = PropPositions.LEFT;
            } else if (largestContourX > right.getAsDouble()) {
                propPosition = PropPositions.RIGHT;
            } else {
                propPosition = PropPositions.MIDDLE;
            }

            // if we have found a new prop position, and it is not unfound, updates the recorded position,
            // this makes sure that if our camera is playing up, we only need to see the prop in the correct position
            // and we will hold onto it
            if (propPosition != previousPropPosition && propPosition != PropPositions.UNFOUND) {
                recordedPropPosition = propPosition;
            }

            // updates the previous prop position to help us check for changes
            previousPropPosition = propPosition;

            // returns back the edited image, don't worry about this too much
            return frame;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // this method draws the rectangle around the largest contour and puts the current prop position into that rectangle
            // you don't need to call it

            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                canvas.drawLines(new float[]{rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx}, linePaint);
            }

            // if the contour exists, draw a rectangle around it and put its position in the middle of the rectangle
            if (largestContour != null) {
                Rect rect = Imgproc.boundingRect(largestContour);

                float[] points = {rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx};

                canvas.drawLine(points[0], points[1], points[0], points[3], linePaint);
                canvas.drawLine(points[0], points[1], points[2], points[1], linePaint);

                canvas.drawLine(points[0], points[3], points[2], points[3], linePaint);
                canvas.drawLine(points[2], points[1], points[2], points[3], linePaint);

                String text = String.format(Locale.ENGLISH, "%s", recordedPropPosition.toString());
            }
        }
*/
    }

}
