package com.company.results;

import org.opencv.core.Mat;

public class RectifyResult {
    public Mat T1;
    public Mat T2;
    public Mat Pn1;
    public Mat Pn2;

    public RectifyResult(Mat T1, Mat T2, Mat Pn1, Mat Pn2){
        this.T1 = T1;
        this.T2 = T2;
        this.Pn1 = Pn1;
        this.Pn2 = Pn2;
    }
}
