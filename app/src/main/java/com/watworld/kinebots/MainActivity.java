package com.watworld.kinebots;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraActivity;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceView;
import android.view.WindowManager;

import java.util.Collections;
import java.util.List;

public class MainActivity extends CameraActivity implements CvCameraViewListener2,
        SensorEventListener {
    private String TAG = "OCVSample::Activity";

    private CameraBridgeViewBase mOpenCvCameraView;
    private Mat processedFrame;

    private SensorManager sensorManager;
    private final float[] accelerometerReading = new float[3];
    private final float[] magnetometerReading = new float[3];

    private final float[] rotationMatrix = new float[9];
    private final float[] orientationAngles = new float[3];

    private int count = 0;

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public MainActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.tutorial1_surface_view);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial1_activity_java_surface_view);

        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);

        mOpenCvCameraView.setCvCameraViewListener(this);

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Do something here if sensor accuracy changes.
        // You must implement this callback in your code.
    }

    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        // Don't receive any more updates from either sensor.
        sensorManager.unregisterListener(this);
    }

    @Override
    public void onResume()
    {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        // Get updates from the accelerometer and magnetometer at a constant rate.
        // To make batch operations more efficient and reduce power consumption,
        // provide support for delaying updates to the application.
        //
        // In this example, the sensor reporting delay is small enough such that
        // the application receives an update before the system checks the sensor
        // readings again.
        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if (accelerometer != null) {
            sensorManager.registerListener(this, accelerometer,
                    SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        }
        Sensor magneticField = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        if (magneticField != null) {
            sensorManager.registerListener(this, magneticField,
                    SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
        }
    }

    @Override
    protected List<? extends CameraBridgeViewBase> getCameraViewList() {
        return Collections.singletonList(mOpenCvCameraView);
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        processedFrame = new Mat();
    }

    public void onCameraViewStopped() {
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        garbageCollection();
        Mat mRgba = inputFrame.rgba();
        Mat mGray = inputFrame.gray();
        Mat cannyEdges = new Mat();
        Mat lines = new Mat();
        Imgproc.resize(mGray, mGray, new Size(1280, 720));
        Imgproc.GaussianBlur(mGray, mGray, new Size(5, 5), 0, 0);
        Imgproc.Canny(mGray, cannyEdges, 50, 100, 3);
        Imgproc.HoughLinesP(cannyEdges, lines, 1, Math.PI / 180, 20, 200, 60);

        for (int i = 0; i < lines.rows(); i++) {
            double[] points = lines.get(i, 0);
            double x1, y1, x2, y2;

            x1 = points[0];
            y1 = points[1];
            x2 = points[2];
            y2 = points[3];

            Point pt1 = new Point(x1*1.5, y1*1.5);
            Point pt2 = new Point(x2*1.5, y2*1.5);

            Imgproc.line(mRgba, pt1, pt2, new Scalar(0, 255, 0), 2);
        }
        Point center = new Point(1920 / 2, 1080 / 2);
        Imgproc.circle(mRgba, center, 3, new Scalar(255, 0, 0));

        mRgba.copyTo(processedFrame);
        updateOrientationAngles();
        double[] pos = getPos(lines);
        Imgproc.putText(processedFrame, orientationAngles[0] + " " + orientationAngles[1] + " " +
                        orientationAngles[2], new Point(500, 30),
                Imgproc.FONT_HERSHEY_SIMPLEX, 1f, new Scalar(255, 0, 0), 2);
        Imgproc.putText(processedFrame, pos[0] + " " + pos[1] + " " +
                        pos[2], new Point(500, 1000),
                Imgproc.FONT_HERSHEY_SIMPLEX, 1f, new Scalar(255, 0, 0), 2);

        return processedFrame;
    }

    // Get readings from accelerometer and magnetometer. To simplify
    // calculations, consider storing these readings as unit vectors.
    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, accelerometerReading,
                    0, accelerometerReading.length);
        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            System.arraycopy(event.values, 0, magnetometerReading,
                    0, magnetometerReading.length);
        }
    }

    // Compute the three orientation angles based on the most recent readings
    // from the device's accelerometer and magnetometer.
    public void updateOrientationAngles() {
        // Update rotation matrix, which is needed to update orientation angles.
        SensorManager.getRotationMatrix(rotationMatrix, null,
                accelerometerReading, magnetometerReading);

        // "mRotationMatrix" now has up-to-date information.

        SensorManager.getOrientation(rotationMatrix, orientationAngles);

        // "mOrientationAngles" now has up-to-date information.

        // Post-processing.
        orientationAngles[0] = addEulerAngle(-orientationAngles[0],
                -(float)Math.PI / 2);
        orientationAngles[1] = -orientationAngles[1];
        orientationAngles[2] = addEulerAngle(orientationAngles[2],
                (float)Math.PI / 2);
    }

    void garbageCollection() {
        count++;
        if (count == 100) {
            System.gc();  // Not a good solution to the memory leak. Might change this later.
            System.runFinalization();
            count = 0;
        }
    }

    float addEulerAngle(float firstVal, float secondVal) {
        firstVal += secondVal;
        if (firstVal > Math.PI) {
            firstVal -= 2 * Math.PI;
        } else if (firstVal < -Math.PI) {
            firstVal += 2 * Math.PI;
        }
        return firstVal;
    }

    static {
        System.loadLibrary("watworld");
    }

    private native double[] getPos(Mat lines);
}
