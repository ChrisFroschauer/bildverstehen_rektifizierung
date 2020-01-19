package com.company;

import com.company.exceptions.WritingImageException;
import com.company.results.RectificationResult;
import com.company.results.RectifyResult;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import static com.company.Epipolar.MIN_NUMBER_POINTS;
import static com.company.Main.*;

public class Rectification {

    /**
     *
     * @param image1 first image
     * @param image2 second image
     * @param PPM1 projection matrix of first image
     * @param PPM2 projection matrix of second image
     * @return a collection containing the two rectified images (TODO: for now fixed size of 2400x1600)
     */
    public static RectificationResult doRectification(Mat image1, Mat image2, int index1, int index2, Mat PPM1, Mat PPM2, MatOfPoint2f imagePoints1, MatOfPoint2f imagePoints2) throws WritingImageException {
        if (imagePoints1.toList().size() < MIN_NUMBER_POINTS || imagePoints2.toList().size() < MIN_NUMBER_POINTS){
            throw new IllegalArgumentException("At least 8 points in imagePoints1 and imagePoints2");
        }

        // ## Testbilderschießen (mit bekanntem abstand und winkel),
        // ## extrinsische Ausrechnen ?

        // - algorithm in Java implementieren

        // call rectify()
        System.out.println("###################### Pre Rectify    ###########");
        System.out.println("Projection Matrix 1 PPM1: \n" + PPM1.dump());
        System.out.println("Projection Matrix 2 PPM2: \n" + PPM2.dump());

        RectifyResult rectifyResult = rectify(PPM1, PPM2);
        System.out.println("###################### Rectify Result ###########");
        System.out.println("T1: " + rectifyResult.T1.dump());
        System.out.println("T2: " + rectifyResult.T2.dump());
        System.out.println("Pn1: " + rectifyResult.Pn1.dump());
        System.out.println("Pn2: " + rectifyResult.Pn2.dump());

        // Normalize T1 and T2 for last value = 1
        /*Mat T1_norm = new Mat();
        Mat T2_norm = new Mat();
        rectifyResult.T1.convertTo(T1_norm, rectifyResult.T1.type(), 1/rectifyResult.T1.get(2,2)[0]);
        rectifyResult.T2.convertTo(T2_norm, rectifyResult.T2.type(), 1/rectifyResult.T2.get(2,2)[0]);
        System.out.println("T1_norm: " + T1_norm.dump());
        System.out.println("T2_norm: " + T2_norm.dump());*/

        // Transform the image pair:
        Mat rectifiedImage1 = new Mat();
        Mat rectifiedImage2 = new Mat();
        Imgproc.warpPerspective(image1, rectifiedImage1, rectifyResult.T1, new Size(2400, 1600));//images.get(firstImage_index).size());
        Imgproc.warpPerspective(image2, rectifiedImage2, rectifyResult.T2, new Size(2400, 1600)); //images.get(secondImage_index).size());
        //showImage(rectifiedImage1, "cali_" + firstImage_index + " Rectifizierung angewandt");
        //showImage(rectifiedImage2, "cali_" + secondImage_index + " Rectifizierung angewandt");

        // Transform the image points:
        MatOfPoint2f imagePointsTransformed1 = new MatOfPoint2f();
        MatOfPoint2f imagePointsTransformed2 = new MatOfPoint2f();

        Core.perspectiveTransform(imagePoints1, imagePointsTransformed1, rectifyResult.T1);
        Core.perspectiveTransform(imagePoints2, imagePointsTransformed2, rectifyResult.T2);

        // Write the rectified images to filesystem:
        String fileImage1 = RECT_IMAGE_OUTPUT_PATH + CAL_IMAGE_NAME + index1 + RECT_IMAGE_SUFFIX + CAL_IMAGE_FORMAT;
        String fileImage2 = RECT_IMAGE_OUTPUT_PATH + CAL_IMAGE_NAME + index2 + RECT_IMAGE_SUFFIX + CAL_IMAGE_FORMAT;
        boolean ok = Imgcodecs.imwrite(fileImage1, rectifiedImage1);
        ok &= Imgcodecs.imwrite(fileImage2, rectifiedImage2);
        if (!ok) {
            throw new WritingImageException("Error: while writing files. (rectified)");
        }

        return new RectificationResult(rectifiedImage1, rectifiedImage2, imagePointsTransformed1, imagePointsTransformed2);
    }

    /**
     * Method implementation after the proposed implementation of:
     * Fusiello et al:
     * A compact algorithm for rectification of stereo pairs
     *
     * @param Po1 projection matrix of the first picture
     * @param Po2 projection matrix of the second picture
     * @return an object containing the relevant information (mainly the new projection matrices Pn1, Pn2 and the homographies T1 and T2 to transform the images)
     */
    public static RectifyResult rectify(Mat Po1, Mat Po2){

        // compute Rectification matrices:
        // skip factorize old PPMs -> got factorization as input
        // decomposeProjectionMatrix Calib3d.decomposeProjectionMatrix();
        Mat A1 = new Mat();
        Mat A2 = new Mat();
        Mat R1 = new Mat();
        Mat R2 = new Mat();
        Mat t1_new = new Mat();
        Mat t2_new = new Mat();
        Calib3d.decomposeProjectionMatrix(Po1, A1, R1, t1_new);
        Calib3d.decomposeProjectionMatrix(Po2, A2, R2, t2_new);

        System.out.println("############ SANITY CHECK: ##########");
        //System.out.println("Intrinsic: \n" + A1.dump());
        //System.out.println("R1: \n" + R1.dump());
        //System.out.println("t1: \n" + t1_new.dump());

        //System.out.println("t1 normalized \n" + (1f/t1_new.get(3,0)[0] * t1_new.get(0,0)[0]));
        /*Sanity check looked pretty close, except t1 is now a 4x1 vector... normalize?
            [-0.02072606854347079;
             0.1111988436950239;
             0.3918417295315831]
            [-0.03584862665327547;
             -0.05034144749231237;
             -0.3725503853238674;
             0.9259518481144094]
         */

        // optical centers
        // c1 = - inv(Po1(:,1:3))*Po1(:,4);
        // c2 = - inv(Po2(:,1:3))*Po2(:,4);
        Mat c1 = new Mat();
        Mat c2 = new Mat();
        Core.gemm(Po1.colRange(new Range(0, 3)).inv(), Po1.col(3), -1, new Mat(), 0, c1);
        Core.gemm(Po2.colRange(new Range(0, 3)).inv(), Po2.col(3), -1, new Mat(), 0, c2);
        //System.out.println("c1:\n" + c1.dump());
        //System.out.println("c2:\n" + c2.dump());

        // new x axis (= direction of the baseline)
        // v1 = (c1-c2);
        Mat v1 = new Mat();
        Core.subtract(c1, c2, v1);
        //System.out.println("v1: \n" + v1.dump());
        // new y axes (orthogonal to new x and old z)
        // v2 = cross(R1(3,:)’,v1);
        Mat R1_row2_transposed = new Mat();
        Core.transpose(R1.row(2), R1_row2_transposed);
        Mat v2 = R1_row2_transposed.cross(v1);
        // new z axes (orthogonal to baseline and y)
        // v3 = cross(v1,v2);
        Mat v3 = v1.cross(v2);
        //System.out.println("v1\n" + v1.dump());
        //System.out.println("v2\n" + v2.dump());
        //System.out.println("v3\n" + v3.dump());

        // new extrinsic parameters
        //R = [v1’/norm(v1)         (Euklidische Norm)
        // v2’/norm(v2)
        // v3’/norm(v3)];
        // translation is left unchanged
        Mat v1_transposed = new Mat();
        Mat v2_transposed = new Mat();
        Mat v3_transposed = new Mat();
        Core.transpose(v1, v1_transposed);
        Core.transpose(v2, v2_transposed);
        Core.transpose(v3, v3_transposed);
        double v1_norm = Core.norm(v1);
        double v2_norm = Core.norm(v2);
        double v3_norm = Core.norm(v3);
        Mat row1 = new Mat();
        Mat row2 = new Mat();
        Mat row3 = new Mat();
        v1_transposed.convertTo(row1, v1_transposed.type(), 1/v1_norm); // = v1' / norm(v1)
        v2_transposed.convertTo(row2, v2_transposed.type(), 1/v2_norm);
        v3_transposed.convertTo(row3, v3_transposed.type(), 1/v3_norm);
        Mat R = new Mat();
        Core.vconcat(List.of(row1, row2, row3), R);
        //System.out.println("row1: \n" + row1.dump());
        System.out.println("R: \n" + R.dump());

        // new intrinsic parameters (arbitrary)
        // A = (A1 + A2)./2;
        // A(1,2)=0; // no skew
        Mat A_sum= new Mat();
        Core.add(A1, A2, A_sum);
        Mat A = new Mat();
        A_sum.convertTo(A, A_sum.type(), 0.5); // A = A_sum / 2
        System.out.println("A: \n" + A.dump());

        // new projection matrices
        // Pn1=A*[R-R*c1 ];
        // Pn2=A*[R-R*c2 ];
        Mat R_times_c1_neg = new Mat();
        Mat R_times_c2_neg = new Mat();
        Core.gemm(R, c1, -1, new Mat(), 0, R_times_c1_neg, 0); // -R * c1
        Core.gemm(R, c2, -1, new Mat(), 0, R_times_c2_neg, 0); // -R * c2
        Mat bracket1 = new Mat();
        Mat bracket2 = new Mat();
        Core.hconcat(List.of(R, R_times_c1_neg), bracket1); // [R | -R*c1]
        Core.hconcat(List.of(R, R_times_c2_neg), bracket2); // [R | -R*c1]
        //WRONG: Core.subtract(R, R_times_c2, bracket2); // [R -R*c2]

        // new projection matrices:
        Mat Pn1 = new Mat();
        Mat Pn2 = new Mat();
        Core.gemm(A, bracket1, 1, new Mat(), 0, Pn1, 0); // A * bracket1
        Core.gemm(A, bracket2, 1, new Mat(), 0, Pn2, 0); // A * bracket2

        // rectifying image transformation
        //System.out.println("PPM1 \n" + PPM1.dump());
        Mat PPM1_sub_col = Po1.colRange(new Range(0, 3));
        Mat PPM2_sub_col = Po2.colRange(new Range(0, 3));
        //System.out.println("PPM1_sub_col \n" + PPM1_sub_col.dump());
        Mat PPM1_sub = PPM1_sub_col.rowRange(new Range(0, 3));
        Mat PPM2_sub = PPM2_sub_col.rowRange(new Range(0, 3));
        //System.out.println("PPM1_sub \n" + PPM1_sub.dump());
        Mat Pn1_sub_col = Pn1.colRange(new Range(0, 3));
        Mat Pn2_sub_col = Pn2.colRange(new Range(0, 3));
        Mat Pn1_sub = Pn1_sub_col.rowRange(new Range(0, 3));
        Mat Pn2_sub = Pn2_sub_col.rowRange(new Range(0, 3));
        Mat T1 = new Mat();
        Mat T2 = new Mat();
        Core.gemm(Pn1_sub, PPM1_sub.inv(), 1, new Mat(), 0, T1, 0); // Pn1(3x3) * inv(Po1(3x3))
        Core.gemm(Pn2_sub, PPM2_sub.inv(), 1, new Mat(), 0, T2, 0); // Pn2(3x3) * inv(Po2(3x3))

        return new RectifyResult(T1, T2, Pn1, Pn2);
    }


    public static RectificationResult rectifyWithOpenCVMethod(Mat image1, Mat image2, int index1, int index2, MatOfPoint2f imagePoints1, MatOfPoint2f imagePoints2) throws WritingImageException{
        if (imagePoints1.toList().size() < MIN_NUMBER_POINTS || imagePoints2.toList().size() < MIN_NUMBER_POINTS){
            throw new IllegalArgumentException("At least 8 points in imagePoints1 and imagePoints2");
        }

        // Find fundamental Matrix
        Mat F = Calib3d.findFundamentalMat(
                new MatOfPoint2f(imagePoints1.submat(0,8, 0,1)),
                new MatOfPoint2f(imagePoints2.submat(0,8, 0,1)));

        // Rectify:
        Mat T1 = new Mat();
        Mat T2 = new Mat();
        Calib3d.stereoRectifyUncalibrated(
                new MatOfPoint2f(imagePoints1.submat(0,8, 0,1)),
                new MatOfPoint2f(imagePoints2.submat(0,8, 0,1)),
                F, image1.size(), T1, T2);

        System.out.println("OpenCV Method T1: \n" + T1.dump());
        System.out.println("OpenCV Method T2: \n" + T2.dump());

        // Transform the image pair:
        Mat rectifiedImage1 = new Mat();
        Mat rectifiedImage2 = new Mat();
        Imgproc.warpPerspective(image1, rectifiedImage1, T1, new Size(2400, 1600));//images.get(firstImage_index).size());
        Imgproc.warpPerspective(image2, rectifiedImage2, T2, new Size(2400, 1600)); //images.get(secondImage_index).size());
        //showImage(rectifiedImage1, "cali_" + firstImage_index + " Rectifizierung angewandt");
        //showImage(rectifiedImage2, "cali_" + secondImage_index + " Rectifizierung angewandt");

        // Transform the image points:
        MatOfPoint2f imagePointsTransformed1 = new MatOfPoint2f();
        MatOfPoint2f imagePointsTransformed2 = new MatOfPoint2f();

        Core.perspectiveTransform(imagePoints1, imagePointsTransformed1, T1);
        Core.perspectiveTransform(imagePoints2, imagePointsTransformed2, T2);

        // Write the rectified images to filesystem:
        String fileImage1 = RECT_IMAGE_OUTPUT_PATH + CAL_IMAGE_NAME + index1 + RECT_IMAGE_SUFFIX + OPEN_CV_SUFFIX + CAL_IMAGE_FORMAT;
        String fileImage2 = RECT_IMAGE_OUTPUT_PATH + CAL_IMAGE_NAME + index2 + RECT_IMAGE_SUFFIX + OPEN_CV_SUFFIX + CAL_IMAGE_FORMAT;
        boolean ok = Imgcodecs.imwrite(fileImage1, rectifiedImage1);
        ok &= Imgcodecs.imwrite(fileImage2, rectifiedImage2);
        if (!ok) {
            throw new WritingImageException("Error: while writing files. (rectified)");
        }

        return new RectificationResult(rectifiedImage1, rectifiedImage2, imagePointsTransformed1, imagePointsTransformed2);
    }

    /*
    public static RectificationResult rectifyWithOpenCVMethod2(Mat image1, Mat image2, int index1, int index2, Mat PPM1, Mat PPM2, Mat distCoeffs, MatOfPoint2f imagePoints1, MatOfPoint2f imagePoints2) throws WritingImageException{
        if (imagePoints1.toList().size() < MIN_NUMBER_POINTS || imagePoints2.toList().size() < MIN_NUMBER_POINTS){
            throw new IllegalArgumentException("At least 8 points in imagePoints1 and imagePoints2");
        }

        Calib3d.stereoRectify(PPM1, distCoeffs, PPM2, distCoeffs, image1.size(), R, t, T1, T2, Pn1, Pn2, Q, 0, -1, new Size(2400, 1600), null, null);

        // Rectify:
        Mat T1 = new Mat();
        Mat T2 = new Mat();
        Calib3d.stereoRectifyUncalibrated(
                new MatOfPoint2f(imagePoints1.submat(0,8, 0,1)),
                new MatOfPoint2f(imagePoints2.submat(0,8, 0,1)),
                F, image1.size(), T1, T2);

        System.out.println("OpenCV Method T1: \n" + T1.dump());
        System.out.println("OpenCV Method T2: \n" + T2.dump());

        // Transform the image pair:
        Mat rectifiedImage1 = new Mat();
        Mat rectifiedImage2 = new Mat();
        Imgproc.warpPerspective(image1, rectifiedImage1, T1, new Size(2400, 1600));//images.get(firstImage_index).size());
        Imgproc.warpPerspective(image2, rectifiedImage2, T2, new Size(2400, 1600)); //images.get(secondImage_index).size());
        //showImage(rectifiedImage1, "cali_" + firstImage_index + " Rectifizierung angewandt");
        //showImage(rectifiedImage2, "cali_" + secondImage_index + " Rectifizierung angewandt");

        // Transform the image points:
        MatOfPoint2f imagePointsTransformed1 = new MatOfPoint2f();
        MatOfPoint2f imagePointsTransformed2 = new MatOfPoint2f();

        Core.perspectiveTransform(imagePoints1, imagePointsTransformed1, T1);
        Core.perspectiveTransform(imagePoints2, imagePointsTransformed2, T2);

        // Write the rectified images to filesystem:
        String fileImage1 = RECT_IMAGE_OUTPUT_PATH + CAL_IMAGE_NAME + index1 + RECT_IMAGE_SUFFIX + OPEN_CV_SUFFIX + CAL_IMAGE_FORMAT;
        String fileImage2 = RECT_IMAGE_OUTPUT_PATH + CAL_IMAGE_NAME + index2 + RECT_IMAGE_SUFFIX + OPEN_CV_SUFFIX + CAL_IMAGE_FORMAT;
        boolean ok = Imgcodecs.imwrite(fileImage1, rectifiedImage1);
        ok &= Imgcodecs.imwrite(fileImage2, rectifiedImage2);
        if (!ok) {
            throw new WritingImageException("Error: while writing files. (rectified)");
        }

        return new RectificationResult(rectifiedImage1, rectifiedImage2, imagePointsTransformed1, imagePointsTransformed2);
    }*/


}
