package com.company;

import com.company.exceptions.ImageBadForCalibrationException;
import com.company.exceptions.WritingImageException;
import com.company.results.CalibrationResult;
import com.company.results.RectificationResult;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;  // keine highgui mehr in Javaimport org.opencv.imgproc.Imgproc;

//showImage:
import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;


public class Main {

    public static final String CAL_IMAGE_PATH = "./res/calibration/";
    public static final String CAL_IMAGE_OUTPUT_PATH = "./res/calibration_output/";
    public static final String CAL_IMAGE_NAME = "cali_";
    public static final String CAL_IMAGE_FORMAT = ".JPG";
    public static final int CAL_IMAGE_NUMBER = 25;
    // for saving images:
    public static final String CORNERS_SUFFIX = "_with_corners";
    public static final String UNDISTORTED_SUFFIX = "_undistorted";

    public static final String RECT_IMAGE_OUTPUT_PATH = "./res/rectification_output/";
    public static final String RECT_IMAGE_SUFFIX = "_rectified";
    public static final String EPIPOLAR_IMAGE_SUFFIX = "_epipolarlines";

    public static final String OPEN_CV_SUFFIX = "OPENCV";

    public static final boolean useUndistorted = false;

    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME); //opencv_java411


        try{
            // Loading images from folder:
            List<Mat> allImages = loadImages();

            int index1 = 2;
            int index2 = 19;

            // Calibrate Camera for all image and calc projection-matrices for images with index1 and incex2
            CalibrationResult calibrationResult = Calibration.calibrate(allImages, index1, index2);

            if (!useUndistorted) {
                // Output images with Epipolarlines:
                Epipolar.outputImagesWithEpipolarLines(
                        allImages.get(index1),
                        allImages.get(index2),
                        index1, index2,
                        calibrationResult.imagePoints1,
                        calibrationResult.imagePoints2,
                        EPIPOLAR_IMAGE_SUFFIX);

                // Rectify the specified images with index1 and index2
                RectificationResult rectified = Rectification.doRectification(
                        allImages.get(index1),
                        allImages.get(index2),
                        index1, index2,
                        calibrationResult.projectionMatrix1,
                        calibrationResult.projectionMatrix2,
                        calibrationResult.imagePoints1,
                        calibrationResult.imagePoints2);
                // Output rectified images with Epipolarlines:
                Epipolar.outputImagesWithEpipolarLines(
                        rectified.rectifiedImage1,
                        rectified.rectifiedImage2,
                        index1, index2,
                        rectified.rectifiedImagePoints1,
                        rectified.rectifiedImagePoints2,
                        EPIPOLAR_IMAGE_SUFFIX + RECT_IMAGE_SUFFIX);
            }else {
                //  Output undistorted images with Epipolarlines:
                Epipolar.outputImagesWithEpipolarLines(
                        calibrationResult.undistortedImage1,
                        calibrationResult.undistortedImage2,
                        index1, index2,
                        calibrationResult.undistortedImagePoints1,
                        calibrationResult.undistortedImagePoints2,
                        EPIPOLAR_IMAGE_SUFFIX);

                // Rectify the specified images with index1 and index2
                RectificationResult rectified = Rectification.doRectification(
                        calibrationResult.undistortedImage1,
                        calibrationResult.undistortedImage2,
                        index1, index2,
                        calibrationResult.projectionMatrix1,
                        calibrationResult.projectionMatrix2,
                        calibrationResult.undistortedImagePoints1,
                        calibrationResult.undistortedImagePoints2);
                // Output rectified images with Epipolarlines:
                Epipolar.outputImagesWithEpipolarLines(
                        rectified.rectifiedImage1,
                        rectified.rectifiedImage2,
                        index1, index2,
                        rectified.rectifiedImagePoints1,
                        rectified.rectifiedImagePoints2,
                        EPIPOLAR_IMAGE_SUFFIX + RECT_IMAGE_SUFFIX);
            }

            // Rectify with OpenCV method:
            RectificationResult rectifiedOpenCV = Rectification.rectifyWithOpenCVMethod(
                    allImages.get(index1),
                    allImages.get(index2),
                    index1, index2,
                    calibrationResult.imagePoints1,
                    calibrationResult.imagePoints2);
            // Output rectified images with Epipolarlines:
            Epipolar.outputImagesWithEpipolarLines(
                    rectifiedOpenCV.rectifiedImage1,
                    rectifiedOpenCV.rectifiedImage2,
                    index1, index2,
                    rectifiedOpenCV.rectifiedImagePoints1,
                    rectifiedOpenCV.rectifiedImagePoints2,
                    EPIPOLAR_IMAGE_SUFFIX + RECT_IMAGE_SUFFIX + OPEN_CV_SUFFIX);


        } catch (FileNotFoundException | ImageBadForCalibrationException | WritingImageException e){
            e.printStackTrace();
        }
    }

    private static List<Mat> loadImages() throws FileNotFoundException{
        List<Mat> images = new ArrayList<>();
        for (int i = 0; i < CAL_IMAGE_NUMBER; i ++) {
            String currImageName = CAL_IMAGE_NAME + i + CAL_IMAGE_FORMAT;

            // Load image
            Mat image = Imgcodecs.imread(CAL_IMAGE_PATH + currImageName);       //".res/cali_test/cali_test_1.JPG");
            if (image.empty()) {
                System.out.println(currImageName + " Error: File empty.");
                throw new FileNotFoundException(CAL_IMAGE_PATH + currImageName);
            }
            images.add(image);
        }
        return images;
    }




    //Similar to android Implementation? https://github.com/opencv/opencv/blob/master/samples/android/camera-calibration/src/org/opencv/samples/cameracalibration/CameraCalibrator.java


    //Codeexample 3 https://www.programcreek.com/java-api-examples/?class=org.opencv.calib3d.Calib3d&method=calibrateCamera
    /*public void calibrate() {
        ArrayList<Mat> rvecs = new ArrayList<Mat>();
        ArrayList<Mat> tvecs = new ArrayList<Mat>();
        Mat reprojectionErrors = new Mat();
        ArrayList<Mat> objectPoints = new ArrayList<Mat>();
        objectPoints.add(Mat.zeros(mCornersSize, 1, CvType.CV_32FC3));
        calcBoardCornerPositions(objectPoints.get(0));
        for (int i = 1; i < mCornersBuffer.size(); i++) {
            objectPoints.add(objectPoints.get(0));
        }

        Calib3d.calibrateCamera(objectPoints, mCornersBuffer, mImageSize,
                mCameraMatrix, mDistortionCoefficients, rvecs, tvecs, mFlags);

        mIsCalibrated = Core.checkRange(mCameraMatrix)
                && Core.checkRange(mDistortionCoefficients);

        mRms = computeReprojectionErrors(objectPoints, rvecs, tvecs, reprojectionErrors);
        Log.i(TAG, String.format("Average re-projection error: %f", mRms));
        Log.i(TAG, "Camera matrix: " + mCameraMatrix.dump());
        Log.i(TAG, "Distortion coefficients: " + mDistortionCoefficients.dump());
    }*/

    //Tutorial from youtube guy
/*
    public void createKnownBoardPosition(Size boardSize, float squareEdgeLength, MatOfPoint2f corners){
        for (int i = 0; i < boardSize.height; i++){
            for (int j = 0; j < boardSize.width; j++){
                corners.push_back(new MatOfPoint3(new Point3(j * squareEdgeLength, i * squareEdgeLength, 0.0)));

            }
        }
    }

    public void getChessboardCorners(List<Mat> images, MatOfPoint2f allFoundCorners){
        for (Mat image : images){

        }
    }*/

    // Tutorial from openCV
   /* public static void calibration(){
        Size boardSize = new Size();
        boardSize.height = 9; //todo look at example xml config
        boardSize.width = 6; //todo look at example xml config
        Size imageSize;

        for(int i = 0; i < CAL_IMAGE_NUMBER; i++) {
            Mat view;
            boolean blinkOutput = false;
            view = Imgcodecs.imread(CAL_IMAGE_PATH + CAL_IMAGE_NAME + i + CAL_IMAGE_FORMAT);

            // check for no more images
            if (view.empty())          // If there are no more images stop the loop
            {
                // if calibration threshold was not reached yet, calibrate now
                if (!imagePoints.empty())
                    // TODO Implement
                    runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints, grid_width,
                            release_object);
                break;
            }
            imageSize = view.size();

            //find pattern

            boolean found;
            int chessBoardFlags = Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE + Calib3d.CALIB_CB_FAST_CHECK; //TODO
            MatOfPoint2f pointBuf = new MatOfPoint2f(); // corners

            found = Calib3d.findChessboardCorners(view, boardSize, pointBuf, chessBoardFlags);

            if (found){
                Mat viewGray = new Mat();
                Imgproc.cvtColor(view, viewGray, Imgproc.COLOR_BGR2GRAY);
                cornerSubPix(viewGray, pointBuf, Size(winSize, winSize),
                        Size(-1, -1), TermCriteria(TermCriteria.COUNT, 30, 0.0001))
            }


        }
    }*/


    public static void showImage(Mat mtrx, String imageTitle) {
        MatOfByte matOfByte = new MatOfByte();  // subclass of org.opencv.core.Mat
        Imgcodecs.imencode(".png", mtrx, matOfByte);
        byte[] byteArray = matOfByte.toArray();
        BufferedImage bufImage = null;  // subclass of java.awt.image
        try {
            InputStream inStream = new ByteArrayInputStream(byteArray);
            bufImage = ImageIO.read(inStream);
        } catch (Exception exc) {
            exc.printStackTrace();
        }
        JFrame frame = new JFrame(imageTitle);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        JLabel imageLabel = new JLabel(new ImageIcon(bufImage));
        frame.getContentPane().add(imageLabel);
        frame.setLocationRelativeTo(null);
        frame.pack();
        frame.setVisible(true);
    }

}
