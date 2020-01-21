package com.company;

import com.company.results.RectificationResult;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

import java.util.ArrayList;
import java.util.List;

public class ExtrinsicFromMeasured {

    public static final int NUMBER_IMAGE_1 = 1337;
    public static final int NUMBER_IMAGE_2 = 1338;

    //TODO doesn't work yet
    public static void extrinsicFromMeasured(Mat intrinsic){

        // Load rubiks images
        Mat image1 = Imgcodecs.imread("./res/custom_extrinsic_matrix_input/rubiks_1.JPG");
        Mat image2 = Imgcodecs.imread("./res/custom_extrinsic_matrix_input/rubiks_2.JPG");
        MatOfPoint2f imagePoints1 = new MatOfPoint2f();
        MatOfPoint2f imagePoints2 = new MatOfPoint2f();

        if (image1.empty() || image2.empty()) {
            System.out.println("Error: File empty. Custom extrinsic");
            return;
        }

        // image 1Rubiks Points, measured in Paint:
        List<Point> pointList1 = new ArrayList<>();
        pointList1.add(new Point(154, 113));
        pointList1.add(new Point(148, 416));

        pointList1.add(new Point(296, 279));
        pointList1.add(new Point(294,552));

        pointList1.add(new Point(410,188));
        pointList1.add(new Point(405,424));

        pointList1.add(new Point(499,318));
        pointList1.add(new Point(493,532));
        imagePoints1.fromList(pointList1);

        // image2 Rubiks Points, measured in Paint:
        List<Point> pointList2 = new ArrayList<>();
        pointList2.add(new Point(552, 250));
        pointList2.add(new Point(548, 418));

        pointList2.add(new Point(615,329));
        pointList2.add(new Point(608,510));

        pointList2.add(new Point(689, 236));
        pointList2.add(new Point(686, 430));

        pointList2.add(new Point(776, 332));
        pointList2.add(new Point(766, 535));
        imagePoints2.fromList(pointList2);


        try{
            Epipolar.outputImagesWithEpipolarLines(
                    image1, image2,
                    NUMBER_IMAGE_1, NUMBER_IMAGE_2,
                    imagePoints1, imagePoints2,
                    "RUBIKS");
            List<Mat> ppms = ppmWithRubiksCubePictureMeasuredExtrinsic(intrinsic);
            RectificationResult rectificationResult = Rectification.doRectification(image1, image2, 1337, 1338, ppms.get(0), ppms.get(1), imagePoints1, imagePoints2);
            Epipolar.outputImagesWithEpipolarLines(
                    rectificationResult.rectifiedImage1, rectificationResult.rectifiedImage2,
                    NUMBER_IMAGE_1, NUMBER_IMAGE_2,
                    rectificationResult.rectifiedImagePoints1, rectificationResult.rectifiedImagePoints2,
                    "RUBIKS_RECTIFIED");

        }catch(Exception e){
            e.printStackTrace();
        }

    }


    private static List<Mat> ppmWithRubiksCubePictureMeasuredExtrinsic(Mat intrinsic){
        // camera 2 is moved -200 mm in x, 100 mm in z and rotated around y by -90° = -pi/2

        // Rotation camera:
        Mat R1 = new Mat(3,3, 0);
        R1.put(0,0,1);
        R1.put(0, 1, 0);
        R1.put(0, 2, 0);
        R1.put(1, 0, 0);
        R1.put(1, 1, 1);
        R1.put(1, 2, 0);
        R1.put(2, 0, 0);
        R1.put(2, 1, 0);
        R1.put(2, 2, 1);
        Mat R2 = new Mat(3,3,0);
        R2.put(0,0,0); // cos(-pi/2)
        R2.put(0, 1, 0);
        R2.put(0, 2, -1); // sin(-pi/2)
        R2.put(1, 0, 0);
        R2.put(1, 1, 1);
        R2.put(1, 2, 0);
        R2.put(2, 0, 1); // -sin(-pi/2)
        R2.put(2, 1, 0);
        R2.put(2, 2, 0); // cos(-pi/2)

        // translation vector
        Mat T1 = new Mat(3,1,0);
        T1.put(0,0, 0);
        T1.put(1,0, 0);
        T1.put(2,0, 0);
        double m_x = 1200/0.0235;     //TODO 1200/25,3mm
        Mat T2 = new Mat(3,1,0);
        T2.put(0,0,-0.200 * m_x);
        T2.put(0,0,0);
        T2.put(0,0,0.100 * m_x);
        // t1 = -R1_1_transposed * T1
        // t2 = -R2_1_transposed * T2
        Mat t1 = new Mat();
        Mat t2 = new Mat();
        Mat R1_1_tranposed = new Mat(3,1,0);
        Mat R2_1_tranposed = new Mat(3,1,0);
        System.out.println(R1.get(0,0));
        R1_1_tranposed.put(0, 0, R1.get(0, 0));
        R1_1_tranposed.put(1, 0, R1.get(0, 1));
        R1_1_tranposed.put(2, 0, R1.get(0, 2));
        R2_1_tranposed.put(0, 0, R2.get(0, 0));
        R2_1_tranposed.put(1, 0, R2.get(0, 1));
        R2_1_tranposed.put(2, 0, R2.get(0, 2));
        //Core.transpose(new Mat(R1, new Range(0,1), new Range(0,3)), R1_1_tranposed);
        //Core.transpose(new Mat(R2, new Range(0,1), new Range(0,3)), R2_1_tranposed);
        Core.gemm(R1_1_tranposed, T1,-1, new Mat(), 0, t1);
        Core.gemm(R2_1_tranposed, T1,-1, new Mat(), 0, t1);

        Mat Rt1 = new Mat();
        Mat Rt2 = new Mat();
        Core.hconcat(List.of(R1, t1), Rt1);
        Core.hconcat(List.of(R2, t2), Rt2);
        // get Projection matrix: PPM = intrinsic * extrinsic = intrinsic * [R|t]
        Mat PPM1 = new Mat();
        Mat PPM2 = new Mat();
        Core.gemm(intrinsic, Rt1, 1, new Mat(), 0, PPM1, 0);
        Core.gemm(intrinsic, Rt2, 1, new Mat(), 0, PPM2, 0);

        return List.of(PPM1, PPM2);
    }

}
