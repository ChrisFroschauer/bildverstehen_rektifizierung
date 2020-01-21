package com.company;

import com.company.exceptions.ImageBadForCalibrationException;
import com.company.exceptions.WritingImageException;
import com.company.results.CalibrationResult;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static com.company.Epipolar.MIN_NUMBER_POINTS;
import static com.company.Main.*;

public class Calibration {

    // calibration pattern parameter: (chessboard)
    private static final double CALI_SQUARE_DIMENSION = 0.025; // in Meter, Messung 06.01.2020: 25mm
    private static final Size CHESSBOARD_DIMENSION = new Size(6, 9);
    // parameter for improved calibration:
    private static final Size IMPROVED_CALIBRATION_WINDOW_SIZE = new Size(11,11);

    //Tutorial https://opencv-java-tutorials.readthedocs.io/en/latest/09-camera-calibration.html
    // TODO: verify the calibration with the rvecs and tvecs: Multiply the object points with the Camera Matrix (A[R|t]) to get the corresponding image point

    /**
     * All the images are supposed to be the same size.
     * The images are assumed to be named CAL_IMAGE_NAME + number + CAL_IMAGE_FORMAT. While number starts at 0 and goes up to (CAL_IMAGE_NUMBER-1).
     *
     * @param images all images
     * @param indexFirst index in List of the first picture you want in your result
     * @param indexSecond index in List of the second picture you want in your result
     * @throws ImageBadForCalibrationException
     */
    public static CalibrationResult calibrate(List<Mat> images, int indexFirst, int indexSecond) throws ImageBadForCalibrationException, WritingImageException {
        // init the imagePoints and objectPoints array:
        List<MatOfPoint2f> imagePoints = new ArrayList<>();
        List<Mat> objectPoints = new ArrayList<>();

        // find handle chessboard corner for each image
        for (int i = 0; i < images.size(); i++){
            Mat image = images.get(i);
            // make it grey
            Mat grayimg = new Mat();
            Imgproc.cvtColor(image, grayimg, Imgproc.COLOR_BGR2GRAY);
            handleChessboardCorners(grayimg, image, imagePoints, objectPoints, i, true);
        }

        // calibrate
        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();

        // init intrinsic with know values
        Mat intrinsic = new Mat();
        intrinsic.put(0,0,1);
        intrinsic.put(1,1,1);

        Mat distCoeffs = new Mat();

        // convert imagePoints to Mat
        List<Mat> imagePointsForCalibration = new ArrayList<>();
        imagePointsForCalibration.addAll(imagePoints);

        // calibrate camera:
        Calib3d.calibrateCamera(objectPoints, imagePointsForCalibration, images.get(0).size(), intrinsic, distCoeffs, rvecs, tvecs);

        // Print out calibration:
        System.out.println("Intrinsic matrix: " + intrinsic.dump());
        System.out.println("dist Coeffs: " + distCoeffs.dump());

        // Undistort all images
        for (int i = 0; i < images.size(); i ++){
            Mat undistorted = new Mat();

            String currImageName = CAL_IMAGE_NAME + i + CAL_IMAGE_FORMAT;
            Mat image = images.get(i);

            // undistort
            Calib3d.undistort(image, undistorted, intrinsic, distCoeffs);

            // save undistorted image:
            System.out.println(currImageName + " Writing undistorted.");
            boolean ok = Imgcodecs.imwrite(CAL_IMAGE_OUTPUT_PATH + CAL_IMAGE_NAME + i + UNDISTORTED_SUFFIX + CAL_IMAGE_FORMAT, undistorted);
            if (!ok) {
                throw new WritingImageException("Error: while writing file. (undistorted)");
            }

        }

        // Calculate Projection Matrix for image with indexFirst and indexSecond
        // get real R:
        Mat r1 = rvecs.get(indexFirst);
        Mat r2 = rvecs.get(indexSecond);
        Mat R1 = new Mat();
        Mat R2 = new Mat();
        Calib3d.Rodrigues(r1, R1);
        Calib3d.Rodrigues(r2, R2);
        System.out.println("R1: " + R1.dump());
        // get [R|t]:
        Mat t1 = tvecs.get(indexFirst);
        Mat t2 = tvecs.get(indexSecond);
        Mat Rt1 = new Mat();
        Mat Rt2 = new Mat();
        Core.hconcat(List.of(R1, t1), Rt1);
        Core.hconcat(List.of(R2, t2), Rt2);
        System.out.println("Rt1: " + Rt1.dump());
        // get Projection matrix: PPM = intrinsic * extrinsic = intrinsic * [R|t]
        Mat PPM1 = new Mat();
        Mat PPM2 = new Mat();
        Core.gemm(intrinsic, Rt1, 1, new Mat(), 0, PPM1, 0);
        Core.gemm(intrinsic, Rt2, 1, new Mat(), 0, PPM2, 0);

        verifyProjectionMatrix( imagePoints.get(indexFirst), imagePoints.get(indexSecond), objectPoints.get(indexFirst), objectPoints.get(indexSecond), PPM1, PPM2);

        // calc undistorted Image Points:
        // undistort
        Mat undistortedImage1 = new Mat();
        Mat undistortedImage2 = new Mat();
        Calib3d.undistort(images.get(indexFirst), undistortedImage1, intrinsic, distCoeffs);
        Calib3d.undistort(images.get(indexSecond), undistortedImage2, intrinsic, distCoeffs);
        MatOfPoint2f undistortedPoints1 = new MatOfPoint2f();
        MatOfPoint2f undistortedPoints2 = new MatOfPoint2f();
        Calib3d.undistortPoints(imagePoints.get(indexFirst), undistortedPoints1, intrinsic, distCoeffs, new Mat(), intrinsic);
        Calib3d.undistortPoints(imagePoints.get(indexSecond), undistortedPoints2, intrinsic, distCoeffs, new Mat(), intrinsic);

        //System.out.println("Undistorted Points1: \n" + undistortedPoints1.dump());
        //System.out.println("Undistorted Points2: \n" + undistortedPoints2.dump());
        //System.out.println("Distorted Points1: \n" + imagePoints.get(indexFirst).dump());
        //System.out.println("Distorted Points2: \n" + imagePoints.get(indexSecond).dump());

        return new CalibrationResult(PPM1, PPM2, indexFirst, indexSecond, intrinsic, imagePoints, distCoeffs, undistortedImage1, undistortedImage2, undistortedPoints1, undistortedPoints2);
    }

    private static void verifyProjectionMatrix(Mat imagePoints1, Mat imagePoints2, Mat objectPoints1, Mat objectPoints2, Mat PPM1, Mat PPM2){
        // Verify PPM1:
        Mat destinationImagePoints1 = new Mat();
        Core.perspectiveTransform(objectPoints1, destinationImagePoints1, PPM1);

        boolean withinRange1 = true;
        for (int i = 0; i < MIN_NUMBER_POINTS; i++){
            double ratio1 = imagePoints1.get(i, 0)[0]/destinationImagePoints1.get(i,0)[0];
            double ratio2 = imagePoints1.get(i, 0)[1]/destinationImagePoints1.get(i,0)[1];
            if (ratio1 > 1.01 || ratio1 < 0.99 || ratio2 > 1.01 || ratio2 < 0.99){
                withinRange1 = false;
            }
        }
        //System.out.println("Found image Points1: \n" + imagePoints1.dump());
        //System.out.println("Calculated image Points1: \n" + destinationImagePoints1.dump());
        if (!withinRange1){
            System.out.println("WARNING: PPM1 is not within 1% error for projection the objectPoints onto the imagePoints");
        }

        // Verify PPM2:
        Mat destinationImagePoints2 = new Mat();
        Core.perspectiveTransform(objectPoints2, destinationImagePoints2, PPM2);
        //System.out.println("Found image Points2: \n" + imagePoints2.dump());
        //System.out.println("Calculated image Points2: \n" + destinationImagePoints2.dump());
        boolean withinRange2 = true;
        for (int i = 0; i < MIN_NUMBER_POINTS; i++){
            double ratio1 = imagePoints2.get(i, 0)[0]/destinationImagePoints2.get(i,0)[0];
            double ratio2 = imagePoints2.get(i, 0)[1]/destinationImagePoints2.get(i,0)[1];
            if (ratio1 > 1.01 || ratio1 < 0.99 || ratio2 > 1.01 || ratio2 < 0.99){
                withinRange2 = false;
            }
        }
        if (!withinRange2){
            System.out.println("WARNING: PPM2 is not within 1% error for projection the objectPoints onto the imagePoints");
        }
    }

    private static void handleChessboardCorners(Mat grayimg, Mat originalImage, List<MatOfPoint2f> imagePoints, List<Mat> objectPoints, int currInt, boolean useImprovedCalibration) throws ImageBadForCalibrationException, WritingImageException{
        String currImageName = CAL_IMAGE_NAME + currInt + CAL_IMAGE_FORMAT;

        //current objectPoints:
        Mat obj = new Mat();
        for (int i = 0; i < CHESSBOARD_DIMENSION.height; i++){
            for (int j = 0; j < CHESSBOARD_DIMENSION.width; j++){
                obj.push_back(new MatOfPoint3f(new Point3(j * CALI_SQUARE_DIMENSION, i * CALI_SQUARE_DIMENSION, 0)));
            }
        }
        MatOfPoint2f imageCorners = new MatOfPoint2f(); // Buffer for the corners
        boolean found = Calib3d.findChessboardCorners(grayimg, CHESSBOARD_DIMENSION, imageCorners,  Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE + Calib3d.CALIB_CB_FAST_CHECK);
        System.out.println(currImageName + " Found: " + found);

        if (found){
            Point[] arr = imageCorners.toArray();
            System.out.print(currImageName + " ImageCorners: [");
            for (Point p : arr){
                System.out.print("[" + p.x + ", " + p.y + "], " );
            }
            System.out.println("]");

            // improve calibration results with:
            if (useImprovedCalibration) {
                TermCriteria term = new TermCriteria(TermCriteria.EPS | TermCriteria.MAX_ITER, 30, 0.1);
                Imgproc.cornerSubPix(grayimg, imageCorners, IMPROVED_CALIBRATION_WINDOW_SIZE, new Size(-1, -1), term);
            }
            // draw corners
            Calib3d.drawChessboardCorners(originalImage, CHESSBOARD_DIMENSION, imageCorners, found);

            // save image with corners
            String out = CAL_IMAGE_OUTPUT_PATH + CAL_IMAGE_NAME + currInt + CORNERS_SUFFIX + CAL_IMAGE_FORMAT;
            System.out.println(out);
            boolean ok = Imgcodecs.imwrite(out, originalImage); //"./res/cali_test_output/cali_test_1_with_corners.jpg", originalImage);
            if (!ok) {
                throw new WritingImageException("Error: while writing file. (with chessboard corners)");
            }

            // add to imagePoints and objectPoints
            imagePoints.add(imageCorners);
            objectPoints.add(obj);
        }else{
            throw new ImageBadForCalibrationException("Corners of calibration image not found in image with index: " + currInt);
        }

    }

}
