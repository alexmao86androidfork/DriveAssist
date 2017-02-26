package riteh.driveassist;

import android.graphics.ImageFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.util.Size;
import android.view.WindowManager;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import static org.opencv.imgproc.Imgproc.line;

public class MainActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener {

    private JavaCameraView mCameraVeiw;

    private BaseLoaderCallback mOpenCVLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case BaseLoaderCallback.SUCCESS: {
                    l("OpenCV Java Library successfully loaded");
                    System.loadLibrary("DriveAssist");
                    l("Loaded DriveAssist library");

                    setupCameraView();

                    mCameraVeiw.enableView();
                    break;
                }
                case BaseLoaderCallback.INIT_FAILED: {
                    l("OpenCV Java library load FAILED");
                    break;
                }
                default: {
                    l("OpenCV Java library ERROR!");
                }
            }
            super.onManagerConnected(status);
        }
    };

    /*
    Setup camera view to use back camera and optimal resolution
     */
    void setupCameraView() {

        String foundCamera = null;
        Size bestResolution = null;
        // Find rear facing camera
        CameraManager cameraManager = (CameraManager) getSystemService(CAMERA_SERVICE);
        try {
            for (String cameraId : cameraManager.getCameraIdList()) {
                CameraCharacteristics camInfo = cameraManager.getCameraCharacteristics(cameraId);

                Integer lensDir = camInfo.get(CameraCharacteristics.LENS_FACING);
                if (lensDir != null && lensDir == CameraCharacteristics.LENS_FACING_FRONT) {
                    continue;
                }

                // Get frame size
                StreamConfigurationMap camConfig = camInfo.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                if (camConfig == null) {
                    continue;
                }

                Size[] availableResolutions = camConfig.getOutputSizes(ImageFormat.YUV_420_888);
                bestResolution = Collections.max(
                    Arrays.asList(availableResolutions),
                    new Comparator<Size>() {
                        @Override
                        public int compare(Size lhs, Size rhs) {
                            int lhsArea = lhs.getHeight() * lhs.getWidth();
                            int rhsArea = rhs.getHeight() * rhs.getWidth();
                            int tgtArea = 76800; //320x240
                            return Math.abs(rhsArea - tgtArea) - Math.abs(lhsArea - tgtArea);
                        }
                    }
                );

                foundCamera = cameraId;
                break;
            }
        } catch (CameraAccessException e) {
            l("Error accessing camera");
            e.printStackTrace();
        }

        if (foundCamera == null) {
            l("No rear facing camera found");
        }
        else {
            mCameraVeiw.setCameraIndex(Integer.valueOf(foundCamera));
            mCameraVeiw.setMaxFrameSize(bestResolution.getWidth(), bestResolution.getHeight());
        }
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        l("Camera view started");
    }

    @Override
    public void onCameraViewStopped() {
        l("Camera view stopped");
    }

    @Override
    public Mat onCameraFrame(Mat inputFrame) {
        long timeStart = System.currentTimeMillis();

        float[] intersections = new float[2];
        float[] slopes = new float[2];

        laneIntersections(inputFrame.getNativeObjAddr(), intersections, slopes);

        int w = inputFrame.cols(), h = inputFrame.rows();
        Point p1 = new Point(intersections[0], h);
        Point p2 = new Point(w / 2, h + (w/2 - intersections[0])*slopes[0]);
        line(inputFrame, p1, p2, new Scalar(255, 0, 0), 1);
        p1 = new Point(w/2, h - (intersections[1] - w/2)*slopes[1]);
        p2 = new Point(intersections[1], h);
        line(inputFrame, p1, p2, new Scalar(255, 0, 0), 1);

        long timeEnd = System.currentTimeMillis();

        l("Intersections " + intersections[0] + ", " + intersections[1]);
        l("Slopes " + slopes[0] + ", " + slopes[1]);

        l("TIMEPERFRAME " + (timeEnd - timeStart));
        return inputFrame;
    }

    private void l(String logString) {
        Log.d("MYDBG", logString);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        mCameraVeiw = (JavaCameraView) this.findViewById(R.id.cameraOutput);
        mCameraVeiw.setVisibility(CameraBridgeViewBase.VISIBLE);
        mCameraVeiw.setCvCameraViewListener(this);
    }

    @Override
    public void onResume() {
        super.onResume();

        if (!OpenCVLoader.initDebug()) {
            l("Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_2_0, this, mOpenCVLoaderCallback);
        } else {
            l("OpenCV library found inside package. Using it!");
            mOpenCVLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        if (mCameraVeiw != null) {
            mCameraVeiw.disableView();
        }
    }

    /**
     * A native method that is implemented by the 'DriveAssist' native library,
     * which is packaged with this application.
     */
    public native boolean laneIntersections(
            long inputFrame, // Address is pased as long
            float[] outputIntersectionAddress,
            float[] outputSlopesAddress);
}
