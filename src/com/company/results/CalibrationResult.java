package com.company.results;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;

import java.util.List;

public class CalibrationResult {
    public Mat projectionMatrix1;
    public Mat projectionMatrix2;
    public int index1;
    public int index2;
    public MatOfPoint2f imagePoints1;
    public MatOfPoint2f imagePoints2;

    public Mat undistortedImage1;
    public Mat undistortedImage2;
    public MatOfPoint2f undistortedImagePoints1;
    public MatOfPoint2f undistortedImagePoints2;

    public Mat distCoeffs;
    public Mat intrinsic;

    public CalibrationResult(Mat ppm1, Mat ppm2, int i1, int i2, Mat intrinsic, List<MatOfPoint2f> imagePoints,
                             Mat distCoeffs, Mat undistortedImage1, Mat undistortedImage2, MatOfPoint2f undistortedImagePoints1, MatOfPoint2f undistortedImagePoints2){
        this.projectionMatrix1 = ppm1;
        this.projectionMatrix2 = ppm2;
        this.intrinsic = intrinsic;
        this.index1 = i1;
        this.index2 = i2;
        this.imagePoints1 = imagePoints.get(i1);
        this.imagePoints2 = imagePoints.get(i2);

        this.distCoeffs = distCoeffs;
        this.undistortedImage1 = undistortedImage1;
        this.undistortedImage2 = undistortedImage2;
        this.undistortedImagePoints1 = undistortedImagePoints1;
        this.undistortedImagePoints2 = undistortedImagePoints2;
    }
}
