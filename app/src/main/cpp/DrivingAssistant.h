#ifndef DRIVEASSIST_DRIVINGASSISTANT_H
#define DRIVEASSIST_DRIVINGASSISTANT_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace cv;

typedef struct TrafficLightWScore {
    KeyPoint kp;
    int score;
} TrafficLightWScore;

class DrivingAssistant {

private:

    double mLaneIntersectionLeft;
    double mLaneIntersectionRight;

    int mFrameRescaledWidth = 320;

    float mRoiOffsetLeft;
    float mRoiOffsetTop;
    float mRoiOffsetRight;
    float mRoiOffsetBottom;

    float mLaneRoiWidth;
    float mLaneRoiHeight;
    float mLeftLaneMinAngle;
    float mLeftLaneMaxAngle;
    float mRightLaneMinAngle;
    float mRightLaneMaxAngle;

    bool mLaneDepartureDetected;
    bool mRedLightDetected;

    Ptr<SimpleBlobDetector> mDetector;

    float mBlobPaddinFactorUp;
    float mBlobPaddingFactorSide;
    int mBlobsInTrafficLight;

    Mat *mTemplate1, *mTemplate2, *mTemplate3;

    float mTemplateMatchThreshold;

    std::vector<TrafficLightWScore> mTrafficLightsWScore;
    int mPixelTolerance;

    std::vector<KeyPoint> mVisibleTrafficLights;

public:
    DrivingAssistant(
            float roiOffsetLeft, float roiOffsetTop, float roiOffsetRight, float roiOffsetBottom
            /* red light params... */);

    void update(Mat &frame);

    bool isLaneDeparted();
    bool isRedLightDetected();

private:
// Methods for RED LIGHT DETECTION
    float templateMatchScore(Mat &image_part, Mat &temp);
    Rect getTrafficLightRect(int max_w, int max_h, KeyPoint kp);
    Mat getTrafficLightFromKeypoint(Mat &image, KeyPoint kp);
    void detectRedLightsInFrame(Mat &inFrame);
    bool samePoint(KeyPoint kp1, KeyPoint kp2);
// Methods for LANE DETECTION
    void detectLanesInFrame(Mat &frame);
    bool laneIntersections(Mat &frame, Vec2f &intersections, Vec2f &slopes);
    bool detectLaneIntersection(Mat &region, Point region_offset, Size image_size, Vec2f angle_range, float &intersection, float &slope);
};

#endif //DRIVEASSIST_DRIVINGASSISTANT_H
