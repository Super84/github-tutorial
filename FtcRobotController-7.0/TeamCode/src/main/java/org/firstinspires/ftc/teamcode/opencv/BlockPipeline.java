package org.firstinspires.ftc.teamcode.opencv;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BlockPipeline extends OpenCvPipeline {

    //These variables hold each step of the process so we can look at them individually
    Mat blur = new Mat();
    Mat hsv = new Mat();
    Mat threshold = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    Mat output = new Mat();

    //This holds the upper and lower limits for our threshold step
    Scalar lower = new Scalar(10,200,80);
    Scalar upper = new Scalar(30,255,255);

    //This is used for stage switching. It does not currently work.
    Stage currentStage = Stage.FINAL;
    boolean switcher = false;

    //This is the bounding box around the largest yellow blob which we assume to be the team element
    Rect boundingBox = new Rect();

    //This us used for stage switching. It does not currently work.
    enum Stage {
        RAW_IMAGE,
        BLUR,
        THRESHOLD,
        FINAL
    }

    //This is the one method that needs to be implemented when making a pipeline
    //It runs each frame of the camera through a series of steps in order to get information from it
    //As an overview, this pipeline blurs the image, converts the image to HSV, thresholds the image
    //to find the orange bits, applies a contour around the threshold, then finds the largest orange
    //blob to draw a box around. Each step is important here for finding the cubes, however
    //your pipelines may need to add or remove steps for performance or quality needs.
    @Override
    public Mat processFrame(Mat input) {
        //First we blur the input image to help reduce noise. This allows tiny orange bits (1-2 pixels
        //each) to be removed and not mess with our later steps
        Imgproc.GaussianBlur(input, blur, new Size(11,11), 1.0);

        //This step turns our RGB image into an HSV image. Because orange is not pure red, green, or blue,
        //it is difficult to set a filter to just get orange. Instead, we use the HSV color scheme to
        //isolate the orange color with any lighting and shade
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);

        //This is the threshold step. It creates a mask, which is a copy of the picture with only 1s or 0s
        //for the pixels. This mask will be 1 where the values in HSV are between the values given above
        //in the upper and lower steps, and 0 otherwise. This allows us to only grab certain areas
        //of the image and allows our algorithms to work much quicker and better.
        Core.inRange(hsv, lower, upper, threshold);

        //We need to clear the contours from the previous step
        contours.clear();

        //This finds the contours, or edges, from the threshold image. This returns a list of each object
        //the algorithm finds. We can perform a lot of different types of analyses on this list to
        //find what we are looking for
        Imgproc.findContours(threshold, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        //This copies the original image to an output image
        input.copyTo(output);

        //This is the analysis that looks for the largest object in the thresholded image.
        //Essentially we are looking for the largest blob of orange, not worrying about shape.
        //We could look for shapes, but this should work for our purposes
        double maxArea = 0;
        int maxIndex = -1;
        for(int i = 0; i < contours.size(); i++) {
            if(Imgproc.contourArea(contours.get(i)) > maxArea) {
                maxIndex = i;
                maxArea = Imgproc.contourArea(contours.get(i));
            }
        }

        //If we find at least one orange blob, then we want to draw on our output image so we can
        //see our process working in the camera viewport
        if(contours.size() > 0) {
            //This draws all the contours onto our viewport image.
            Imgproc.drawContours(output, contours, -1, new Scalar(255,0,0),5);

            //We want a box around the biggest contour so it is easier to work with. This gives us
            //the smallest box that still contains the entire object. We draw this box on the output
            //in blue, with the centerpoint in green. We also display text along the top with the
            //location of the center point.
            boundingBox = Imgproc.boundingRect(contours.get(maxIndex));
            Imgproc.rectangle(output, boundingBox, new Scalar(0,0,255), 5);
            Imgproc.circle(output, centerOfBox(), 5, new Scalar(0,255,0), -1);
            Imgproc.putText(output, String.format("x: %.1f, y: %.1f", centerOfBox().x, centerOfBox().y),new Point(20,100),Imgproc.FONT_HERSHEY_COMPLEX, 3, new Scalar(0,255,0));
        }

        //This is used for stage switching, it does not currently work. We just want to return the
        //output image that we have been drawing on
        if(switcher) {
            switch(currentStage) {
                case BLUR:
                    return blur;
                case THRESHOLD:
                    return threshold;
                case FINAL:
                    return output;
                default:
                    return input;
            }
        } else {
            return output;
        }
    }

    //This is used for stage switching, it does not currently work.
    @Override
    public void onViewportTapped() {
        int temp = currentStage.ordinal();
        temp = (temp + 1) % Stage.values().length;
        currentStage = Stage.values()[temp];
    }

    //This returns the center point of the box we created in the earlier steps. This is what we grab
    //in our robot code so the robot knows where exactly the block is in the camera view
    public Point centerOfBox() {
        if(boundingBox != null) {
            return new Point((boundingBox.br().x+boundingBox.tl().x)/2, (boundingBox.br().y+boundingBox.tl().y)/2);
        } else return null;

    }

    //Here we can get the full bounding box if we would like
    public Rect getBoundingBox() {
        return boundingBox;
    }

}
