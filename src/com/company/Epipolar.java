package com.company;

import com.company.exceptions.WritingImageException;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import static com.company.Main.*;
import static com.company.Main.CAL_IMAGE_FORMAT;

public class Epipolar {

    public final static int MIN_NUMBER_POINTS = 8;

    public static void outputImagesWithEpipolarLines(Mat image1, Mat image2, int index1, int index2, MatOfPoint2f imagePoints1, MatOfPoint2f imagePoints2, String outputSuffix) throws WritingImageException {
        if (imagePoints1.toList().size() < MIN_NUMBER_POINTS || imagePoints2.toList().size() < MIN_NUMBER_POINTS){
            throw new IllegalArgumentException("At least 8 points in imagePoints1 and imagePoints2");
        }

        // Find fundamental Matrix F (I'm typically simply using the first 8 points of the callibration-pattern)
        Mat F = Calib3d.findFundamentalMat(
                new MatOfPoint2f(imagePoints1.submat(0,8, 0,1)),
                new MatOfPoint2f(imagePoints2.submat(0,8, 0,1)));
        System.out.println("Fundamental Matrix F: \n" + F.dump());

        // Find epipolar lines
        Mat epipolarLines1 = new Mat();
        Mat epipolarLines2 = new Mat();
        Calib3d.computeCorrespondEpilines(imagePoints2.submat(0,8, 0,1), 2, F, epipolarLines1);
        Calib3d.computeCorrespondEpilines(imagePoints1.submat(0,8, 0,1), 1, F, epipolarLines2);

        Mat img1WithLines = drawLines(image1, epipolarLines1);
        Mat img2WithLines = drawLines(image2, epipolarLines2);

        // Write Images with epipolar lines:
        String fileImage1 = RECT_IMAGE_OUTPUT_PATH + CAL_IMAGE_NAME + index1 + outputSuffix + CAL_IMAGE_FORMAT;
        String fileImage2 = RECT_IMAGE_OUTPUT_PATH + CAL_IMAGE_NAME + index2 + outputSuffix + CAL_IMAGE_FORMAT;
        boolean ok = Imgcodecs.imwrite(fileImage1, img1WithLines);
        ok &= Imgcodecs.imwrite(fileImage2, img2WithLines);
        if (!ok) {
            throw new WritingImageException("Error: while writing files. (rectified)");
        }
    }


    /** draw lines to an image.
     *  Source: https://answers.opencv.org/question/52119/strange-epipolar-lines-and-3d-reconstruction-opencv-for-java/
     */
    private static Mat drawLines(Mat image1, Mat lines1){
        Mat resultImg = new Mat();
        image1.copyTo(resultImg);
        //Imgproc.cvtColor(image1, resultImg, Imgproc.COLOR_BGR2GRAY);
        int epiLinesCount = lines1.rows();

        double a, b, c;

        for (int line = 0; line < epiLinesCount; line++) {
            a = lines1.get(line, 0)[0];
            b = lines1.get(line, 0)[1];
            c = lines1.get(line, 0)[2];

            int x0 = 0;
            int y0 = (int) (-(c + a * x0) / b);
            int x1 = resultImg.cols() / 2;
            int y1 = (int) (-(c + a * x1) / b);

            Point p1 = new Point(x0, y0);
            Point p2 = new Point(x1, y1);
            Scalar color = new Scalar(255, 0, 0);
            Imgproc.line(resultImg, p1, p2, color);

        }
        for (int line = 0; line < epiLinesCount; line++) {
            a = lines1.get(line, 0)[0];
            b = lines1.get(line, 0)[1];
            c = lines1.get(line, 0)[2];

            int x0 = resultImg.cols() / 2;
            int y0 = (int) (-(c + a * x0) / b);
            int x1 = resultImg.cols();
            int y1 = (int) (-(c + a * x1) / b);

            Point p1 = new Point(x0, y0);
            Point p2 = new Point(x1, y1);
            Scalar color = new Scalar(255, 0, 0);
            Imgproc.line(resultImg, p1, p2, color);

        }
        return resultImg;
    }

}
