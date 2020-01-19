package com.company.results;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;

public class RectificationResult {

    public Mat rectifiedImage1;
    public Mat rectifiedImage2;

    public MatOfPoint2f rectifiedImagePoints1;
    public MatOfPoint2f rectifiedImagePoints2;

    public RectificationResult(Mat image1, Mat image2, MatOfPoint2f imagePoints1, MatOfPoint2f imagePoints2){
        rectifiedImage1 = image1;
        rectifiedImage2 = image2;
        rectifiedImagePoints1 = imagePoints1;
        rectifiedImagePoints2 = imagePoints2;
    }
}
