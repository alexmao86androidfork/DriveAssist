#ifndef DRIVEASSIST_DRIVINGASSISTANT_H
#define DRIVEASSIST_DRIVINGASSISTANT_H

#include <opencv2/core/core.hpp>
using namespace cv;

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

public:
    DrivingAssistant(
            float roiOffsetLeft, float roiOffsetTop, float roiOffsetRight, float roiOffsetBottom
            /* red light params... */);

    void update(Mat &frame);

    bool isLaneDeparted();
    bool isRedLightDetected();

private:
// Methods for RED LIGHT DETECTION
    // TODO: ...
    void detectRedLightsInFrame(Mat &frame);
// Methods for LANE DETECTION
    void detectLanesInFrame(Mat &frame);
    bool laneIntersections(Mat &frame, Vec2f &intersections, Vec2f &slopes);
    bool detectLaneIntersection(Mat &region, Point region_offset, Size image_size, Vec2f angle_range, float &intersection, float &slope);
};

#endif //DRIVEASSIST_DRIVINGASSISTANT_H
