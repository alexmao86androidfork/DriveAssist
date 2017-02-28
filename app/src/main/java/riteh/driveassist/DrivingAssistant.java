package riteh.driveassist;


import org.opencv.core.Mat;

public class DrivingAssistant {

    public interface LaneDepartureCallback {
        void onLaneDepartureDetected();
    }

    public interface RedLightCallback {
        void onRedLightDetected();
    }

    // Address of native object
    private long mNativeDrivingAssistant;

    private LaneDepartureCallback mOnLaneDepartureCallback;
    private RedLightCallback mOnRedLightDetectedCallback;

    private boolean mLaneDepartureDetected;
    private boolean mRedLightDetected;

    public DrivingAssistant(
            LaneDepartureCallback onLaneDepartureCallback,
            RedLightCallback onRedLightDetectedCallback) {
        mNativeDrivingAssistant = nativeCreateDrivingAssistant();
        mOnRedLightDetectedCallback = onRedLightDetectedCallback;
        mOnLaneDepartureCallback = onLaneDepartureCallback;
    }

    public void update(Mat frame) {
        if (mNativeDrivingAssistant != 0) {
            boolean[] laneDeparted = new boolean[1];
            boolean[] redLightDetected = new boolean[1];

            nativeUpdate(mNativeDrivingAssistant, frame.getNativeObjAddr(), laneDeparted, redLightDetected);

            mLaneDepartureDetected = laneDeparted[0];
            mRedLightDetected = redLightDetected[0];
        }

        if (mLaneDepartureDetected) {
            mOnLaneDepartureCallback.onLaneDepartureDetected();
        }

        if (mRedLightDetected) {
            mOnRedLightDetectedCallback.onRedLightDetected();
        }
    }

    public void close() {
        if (mNativeDrivingAssistant != 0) {
            nativeDestroyDrivingAssistant(mNativeDrivingAssistant);
        }
    }

    private static native long nativeCreateDrivingAssistant();
    private static native void nativeDestroyDrivingAssistant(long drivingAssistantAddress);
    private static native void nativeUpdate(
            long drivingAssistantAddress, long inputFrameAddress,
            boolean[] outLaneDeparted, boolean[] outRedLightDetected);
}
