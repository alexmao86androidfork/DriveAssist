package riteh.driveassist;

import android.graphics.ImageFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.MediaPlayer;
import android.media.SoundPool;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.util.Size;
import android.view.View;
import android.view.WindowManager;
import android.widget.ImageView;

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

    private JavaCameraView mCameraView;
    private DrivingAssistant mDrivingAssistant;

    private BaseLoaderCallback mOpenCVLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case BaseLoaderCallback.SUCCESS: {
                    l("OpenCV Java Library successfully loaded");
                    System.loadLibrary("DriveAssist");
                    l("Loaded DriveAssist library");

                    setupCameraView();

                    mCameraView.enableView();
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
    private ImageView mWarningImage;
    private MediaPlayer mWarningSound;

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
                            int tgtArea = 307200; //640x480
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
            mCameraView.setCameraIndex(Integer.valueOf(foundCamera));
            mCameraView.setMaxFrameSize(bestResolution.getWidth(), bestResolution.getHeight());
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

        mDrivingAssistant.update(inputFrame);

        long timeEnd = System.currentTimeMillis();

        l("TIMEPERFRAME " + (timeEnd - timeStart));
        return inputFrame;
    }

    private void l(String logString) {
        Log.d("MYDBG", logString);
    }

    private void playWarningSound() {
        if (mWarningSound != null) {
            if (!mWarningSound.isPlaying()) {
                mWarningSound.start();
            }
        }
    }

    private void stopWarningSound() {
        if (mWarningSound != null) {
            if (mWarningSound.isPlaying()) {
                mWarningSound.pause();
                mWarningSound.seekTo(0);
            }
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        mCameraView = (JavaCameraView) this.findViewById(R.id.cameraOutput);
        mCameraView.setVisibility(CameraBridgeViewBase.VISIBLE);
        mCameraView.setCvCameraViewListener(this);

        mWarningImage = (ImageView) this.findViewById(R.id.warningImage);
    }

    DrivingAssistant.RedLightCallback mOnRedLightDetected = new DrivingAssistant.RedLightCallback() {
        @Override
        public void onRedLightDetected() {
            l("Red light detected");
        }

        @Override
        public void onRedLightOver() {
            l("Red light no longer in image");
        }
    };

    DrivingAssistant.LaneDepartureCallback mOnLaneDeparture = new DrivingAssistant.LaneDepartureCallback() {
        @Override
        public void onLaneDepartureDetected() {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    l("Lane departure detected");
                    mWarningImage.setVisibility(View.VISIBLE);
                    playWarningSound();
                }
            });
        }

        @Override
        public void onLaneDepartureOver() {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    l("Lane departure over");
                    mWarningImage.setVisibility(View.GONE);
                    stopWarningSound();
                }
            });
        }
    };

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

        mDrivingAssistant = new DrivingAssistant(mOnLaneDeparture, mOnRedLightDetected);

        mWarningSound = MediaPlayer.create(getApplicationContext(), R.raw.warning);
    }

    @Override
    public void onPause() {
        super.onPause();
        if (mCameraView != null) {
            mCameraView.disableView();
        }

        if (mDrivingAssistant != null) {
            mDrivingAssistant.close();
            mDrivingAssistant = null;
        }

        if (mWarningSound != null) {
            mWarningSound.stop();
            mWarningSound.release();
            mWarningSound = null;
        }
    }
}
