package team.spicyketchup.subsystem.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//blue
@Disabled
public class redAudiencePipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        //middle
        RIGHT,
        //left
        NOT_FOUND
        //left
    }




    private Location location = Location.LEFT;
    static final Rect LEFT_ROI = new Rect(
            new Point(30, 98),
            new Point(100, 165));
    static final Rect RIGHT_ROI = new Rect(
            new Point(180, 100),
            new Point(245, 150));
    static double PERCENT_COLOR_THRESHOLD = 0.2;

    public redAudiencePipeline(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //COLOR BGR240,51,100

//        Scalar lowHSV = new Scalar(100,100,100);
//        Scalar highHSV = new Scalar(180,255,255);
        Scalar lowHSV = new Scalar(159,50,70);
        Scalar highHSV = new Scalar(180,255,255);




        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(rightValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneLeft) {
            location = Location.LEFT;
            telemetry.addData("Cube Location", "left");
        }
        else if (stoneRight) {
            location = Location.RIGHT;
            telemetry.addData("Cube Location", "middle");
        }
        else {
            location = Location.NOT_FOUND;
            telemetry.addData("Cube Location", "right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar red = new Scalar(255, 0, 0);
        Scalar green = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? green:red);//middle
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? green:red);//right

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}