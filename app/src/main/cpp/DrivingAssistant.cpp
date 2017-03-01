#include "DrivingAssistant.h"
#include "DriveAssistUtils.h"
#include "TemplateArrays.h"

#include <android/log.h>

#define LOG_D(LOG_TAG, ...)  __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)

DrivingAssistant::DrivingAssistant(
        float roiOffsetLeft, float roiOffsetTop, float roiOffsetRight, float roiOffsetBottom) {
    mRoiOffsetLeft = roiOffsetLeft;
    mRoiOffsetTop = roiOffsetTop;
    mRoiOffsetRight = roiOffsetRight;
    mRoiOffsetBottom = roiOffsetBottom;
    mLaneRoiWidth = mRoiOffsetRight - mRoiOffsetLeft;
    mLaneRoiHeight = mRoiOffsetBottom - mRoiOffsetTop;

    mLaneIntersectionLeft = 0;
    mLaneIntersectionRight = mFrameRescaledWidth;

    mLeftLaneMinAngle = 20;
    mLeftLaneMaxAngle = 70;
    mRightLaneMinAngle = 89 + 20;
    mRightLaneMaxAngle = 179 - 20;

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Filter by Color
    params.filterByColor = true;
    params.blobColor = 255;

    // Filter by Area
    params.filterByArea = true;
    params.minArea = 16;
    params.maxArea = 38400;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.5;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.5;

    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.5;

    mDetector = SimpleBlobDetector::create(params);

    mBlobPaddinFactorUp = 0.2;
    mBlobPaddingFactorSide = 0.2;
    mBlobsInTrafficLight = 3;

    mTemplate1 = new Mat(54, 20, CV_8UC3, tarrays::template1);
    mTemplate2 = new Mat(38, 13, CV_8UC3, tarrays::template2);
    mTemplate3 = new Mat(179, 56, CV_8UC3, tarrays::template3);

    mTemplateMatchThreshold = 0.82;

    mPixelTolerance = 50;
}

void DrivingAssistant::update(Mat &frame) {

    detectLanesInFrame(frame);
    detectRedLightsInFrame(frame);
    drawLanes(frame);
}

// Methods for RED LIGHT DETECTION

float DrivingAssistant::templateMatchScore(Mat &imagePart, Mat &temp) {

    Mat resizedTemplate;
    resize(*mTemplate1, resizedTemplate, imagePart.size(), 0, 0, INTER_LINEAR);

    Mat result = Mat(1, 1, CV_32FC1);

    matchTemplate(imagePart, resizedTemplate, result, TM_CCORR_NORMED);

    return result.at<float>(0, 0);
}

Rect DrivingAssistant::getTrafficLightRect(int max_w, int max_h, KeyPoint kp) {
    int x = std::max(0, int(kp.pt.x - (mBlobPaddingFactorSide + 0.5) * kp.size));
    int y = std::max(0, int(kp.pt.y - (mBlobPaddingFactorSide + 0.5) * kp.size));

    int width = std::min(max_w-x, int(kp.size + 2 * mBlobPaddingFactorSide * kp.size));
    int height = std::min(max_h-y, int((kp.size + 2 * mBlobPaddinFactorUp * kp.size) *
                                               mBlobsInTrafficLight));

    return Rect(x, y, width, height);
}

Mat DrivingAssistant::getTrafficLightFromKeypoint(Mat &image, KeyPoint kp) {
    Rect roi = getTrafficLightRect(image.cols, image.rows, kp);

    return image(roi);
}

bool DrivingAssistant::samePoint(KeyPoint kp1, KeyPoint kp2) {
    return (kp1.pt.x - mPixelTolerance <= kp2.pt.x) && (kp2.pt.x <= kp1.pt.x + mPixelTolerance) &&
            (kp1.pt.y - mPixelTolerance <= kp2.pt.y) && (kp2.pt.y <= kp1.pt.y + mPixelTolerance);
}

void DrivingAssistant::detectRedLightsInFrame(Mat &inFrame) {
    Mat frame;
    cvtColor(inFrame, frame, COLOR_RGBA2BGR);

    // Grab only RED channel
    Mat redChannelRelevant;
    extractChannel(frame, redChannelRelevant, 2);

    // Grab relevant frame part
    redChannelRelevant = redChannelRelevant.rowRange(0, frame.rows/2);

    // Find blobs in this frame (red channel relevant part)
    std::vector<KeyPoint> blobs;
    mDetector->detect(redChannelRelevant, blobs);

    // Confirm blobs are traffic lights (template matching)
    std::vector<KeyPoint>::iterator blob = blobs.begin();
    while(blob != blobs.end()) {
        Mat imagePart = getTrafficLightFromKeypoint(frame, *blob);

        bool isTrafficLight =
                templateMatchScore(imagePart, *mTemplate1) >= mTemplateMatchThreshold ||
                templateMatchScore(imagePart, *mTemplate2) >= mTemplateMatchThreshold ||
                templateMatchScore(imagePart, *mTemplate3) >= mTemplateMatchThreshold;

        if (!isTrafficLight) {
            blob = blobs.erase(blob);
        }else {
            ++blob;
        }
    }

    mVisibleTrafficLights.clear();
    std::vector<TrafficLightWScore>::iterator existingTrafficLight = mTrafficLightsWScore.begin();
    while (existingTrafficLight != mTrafficLightsWScore.end()) {

        std::vector<KeyPoint>::iterator frameTrafficLight = blobs.begin();
        bool exists = false;
        while(frameTrafficLight != blobs.end()) {
            if (samePoint((*existingTrafficLight).kp, *frameTrafficLight)) {
                exists = true;
                (*existingTrafficLight).score = std::min(40, (*existingTrafficLight).score + 5);
                (*existingTrafficLight).kp = *frameTrafficLight;
                frameTrafficLight = blobs.erase(frameTrafficLight);
            } else {
                ++frameTrafficLight;
            }
        }

        if (!exists) {
            (*existingTrafficLight).score--;
            if ((*existingTrafficLight).score <= 0) {
                existingTrafficLight = mTrafficLightsWScore.erase(existingTrafficLight);
                continue;
            }
        }

        if ((*existingTrafficLight).score >= 15) {
            mVisibleTrafficLights.push_back((*existingTrafficLight).kp);
        }
        ++existingTrafficLight;
    }

    for (KeyPoint &remainingBlob : blobs) {
        mTrafficLightsWScore.push_back(TrafficLightWScore{remainingBlob, 5});
    }

    for (KeyPoint &t : mVisibleTrafficLights) {
        circle(inFrame, t.pt, int(t.size)/2, cv::Scalar(255, 0, 0), 1);
        rectangle(inFrame, getTrafficLightRect(inFrame.cols, inFrame.rows, t),
                  cv::Scalar(255, 0, 0));
    }

    mRedLightDetected = !mVisibleTrafficLights.empty();
}

bool DrivingAssistant::isRedLightDetected() {
    return mRedLightDetected;
}

// Methods for LANE DETECTION

void DrivingAssistant::drawLanes(Mat &frame) {
    line(frame,
         cv::Point((int) mIntersections[0], frame.rows),
         cv::Point(frame.cols / 3, frame.rows + int(((frame.cols / 3) - mIntersections[0]) * mSlopes[0])),
         cv::Scalar(0, 255, 0));

    line(frame,
         cv::Point(frame.cols * 2 / 3, int(frame.rows - (mIntersections[1] - (frame.cols * 2 / 3)) * mSlopes[1])),
         cv::Point((int)mIntersections[1], frame.rows),
         cv::Scalar(0, 255, 0));
}

bool DrivingAssistant::isLaneDeparted() {
    return mLaneDepartureDetected;
}

void DrivingAssistant::detectLanesInFrame(Mat &frame) {
    bool lanesDetected = laneIntersections(frame, mIntersections, mSlopes);
    if (lanesDetected)  {
        mLaneIntersectionLeft = 0.8 * mLaneIntersectionLeft + 0.2 * mIntersections[0];
        mLaneIntersectionRight = 0.8 * mLaneIntersectionRight + 0.2 * mIntersections[1];
    }

    if (mLaneIntersectionLeft > frame.cols * 0.25 || mLaneIntersectionRight < frame.cols * 0.75) {
        mLaneDepartureDetected = true;
    }
    else {
        mLaneDepartureDetected = false;
    }
}

bool DrivingAssistant::detectLaneIntersection(Mat &region, Point region_offset, Size image_size, Vec2f angle_range, float &intersection, float &slope)
{
    Mat gray_region;
    cvtColor(region, gray_region, COLOR_RGBA2GRAY);

    Mat thresholded_region; // Unused
    // Perform otsu thresholding to get optimal threshold
    // TODO: figure out how to get value without performing threshold
    double otsu_threshold = threshold(gray_region, thresholded_region, 1, 1, THRESH_OTSU);

    Mat region_edges;
    Canny(gray_region, region_edges, otsu_threshold, otsu_threshold * 0.7);

    std::vector<Vec2f> detected_lines;
    // input_image, detected_lines, rho, theta, threshold
    HoughLines(region_edges, detected_lines, 1, CV_PI / 90, 10);

    float min_angle = fmodf(angle_range[0], 180);
    if (min_angle < 0) min_angle += 180;
    float max_angle = fmodf(angle_range[1], 180);
    if (max_angle < 0) max_angle += 180;

    if (min_angle > max_angle)
    {
        float tmp = max_angle;
        max_angle = min_angle;
        min_angle = tmp;
    }
    else if (min_angle == max_angle)
    {
        min_angle = 0.0;
        max_angle = 180.0;
    }

    min_angle *= dautils::deg2rad;
    max_angle *= dautils::deg2rad;

    std::vector<Vec2f> filtered_lines;
    std::copy_if(detected_lines.begin(), detected_lines.end(), std::back_inserter(filtered_lines), [=](Vec2f line) -> bool
    {
        // Select only lines that have angle in angle_range
        // 0 < angle < pi (0 = vertical, pi/2 = horizontal)
        float angle_radians = line[1];

        if (min_angle > max_angle)
        {
            return angle_radians <= max_angle || angle_radians >= min_angle;
        }
        else
        {
            return min_angle <= angle_radians && angle_radians <= max_angle;
        }
    });

    if (filtered_lines.size() == 0)
    {
        return false;
    }

    Vec2f minmaxangle = Vec2f(filtered_lines[0][1], filtered_lines[0][1]);
    for (auto line : filtered_lines) {
        if (line[1] < minmaxangle[0]) minmaxangle[0] = line[1];
        if (line[1] > minmaxangle[1]) minmaxangle[1] = line[1];
    }
    //std::cout << "Min " << minmaxangle[0] * rad2deg << " Max " << minmaxangle[1] * rad2deg << "\n";

    // First calculate intersection relative to region, then add offset
    int region_0_to_image_bottom = image_size.height - region_offset.y;
    int region_0_to_image_left = region_offset.x;

    Vec2f &strongest_line = filtered_lines[0];
    // rho, theta = line parameters in hough space
    float rho = strongest_line[0];
    float theta = strongest_line[1];

    float cos_theta = cosf(theta) + 0.0000001f;
    float sin_theta = sinf(theta) + 0.0000001f; // Avoid division error

    slope = -cos_theta / sin_theta;
    //float y_intersection = rho / sin_theta;

    // Intersection of image bottom and detected line, in region coordinate space
    float intersection_relative_to_region = (rho - sin_theta * region_0_to_image_bottom) / cos_theta;

    // Intersection in image coordinate space
    float intersection_relative_to_image = intersection_relative_to_region + region_0_to_image_left;
    intersection = intersection_relative_to_image;

    return true;
}


/*
 * Params:
 *  frame - frame on which the detection is performed
 *  intersections - output vector of x values where lanes intersect image at bottom
 * Returns:
 *  true if both lanes are detected, otherwise false
 */
bool DrivingAssistant::laneIntersections(Mat &frame, Vec2f &intersections, Vec2f &slopes)
{
    float ratio;
    Mat frame_small = dautils::resize_keeping_aspect_ratio(frame, mFrameRescaledWidth, ratio);

    int width = frame_small.cols;
    int height = frame_small.rows;

    Rect left_lane_roi = Rect(
            int(width * mRoiOffsetLeft),
            int(height * mRoiOffsetTop),
            int(width * mLaneRoiWidth),
            int(height * mLaneRoiHeight)
    );

    Rect right_lane_roi = Rect(
            frame_small.cols - left_lane_roi.x - left_lane_roi.width,
            left_lane_roi.y,
            left_lane_roi.width,
            left_lane_roi.height
    );


    Mat left_lane = frame_small(left_lane_roi);
    Mat right_lane = frame_small(right_lane_roi);

    float left_intersection_small;
    bool detection_success = detectLaneIntersection(
            left_lane,
            left_lane_roi.tl(),
            Size(width, height),
            Vec2f(mLeftLaneMinAngle, mLeftLaneMaxAngle),//Vec2f(20.0, 70.0),
            left_intersection_small,
            slopes[0]);

    if (!detection_success)
    {
        return false;
    }

    float right_intersection_small;
    detection_success = detectLaneIntersection(
            right_lane,
            right_lane_roi.tl(),
            Size(width, height),
            Vec2f(mRightLaneMinAngle, mRightLaneMaxAngle),
            right_intersection_small,
            slopes[1]);

    if (!detection_success)
    {
        return false;
    }

    intersections[0] = left_intersection_small / ratio;
    intersections[1] = right_intersection_small / ratio;

    return true;
}

