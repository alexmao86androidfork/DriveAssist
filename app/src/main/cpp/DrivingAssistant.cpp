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
    params.minArea = 8;

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

    mTemplate1 = new Mat(54, 20, CV_8UC3, tarrays::template1);

    mTemplate2 = new Mat(38, 13, CV_8UC3, tarrays::template2);

    mTemplate3 = new Mat(179, 56, CV_8UC3, tarrays::template3);
}

void DrivingAssistant::update(Mat &frame) {

    detectLanesInFrame(frame);
    detectRedLightsInFrame(frame);
}

// Methods for RED LIGHT DETECTION

float DrivingAssistant::templateMatchScore(Mat &image_part, Mat &temp) {

    Mat resized_template;
    resize(*mTemplate1, resized_template, image_part.size(), 0, 0, INTER_LINEAR);

    Mat result = Mat(1, 1, CV_32FC1);

    matchTemplate(image_part, resized_template, result, TM_CCORR_NORMED);

    return result.at<float>(0, 0);
}

Mat DrivingAssistant::getTrafficLightFromKeypoint(Mat &image, KeyPoint kp) {
    float BLOB_PADDING_FACTOR_UP = 0.2;
    float BLOB_PADDING_FACTOR_SIDE = 0.2;
    int BLOBS_IN_TRAFFIC_LIGHT = 3;

    int x = std::max(0, int(kp.pt.x - (BLOB_PADDING_FACTOR_SIDE + 0.5) * kp.size));
    int y = std::max(0, int(kp.pt.y - (BLOB_PADDING_FACTOR_SIDE + 0.5) * kp.size));

    int width = std::min(image.cols-x, int(kp.size + 2 * BLOB_PADDING_FACTOR_SIDE * kp.size));
    int height = std::min(image.rows-y, int((kp.size + 2 * BLOB_PADDING_FACTOR_UP + kp.size) *
                                            BLOBS_IN_TRAFFIC_LIGHT));

    Rect roi(x, y, width, height);

    return image(roi);
}

void DrivingAssistant::detectRedLightsInFrame(Mat &in_frame) {
    float TEMPLATE_MATCH_THRESHOLD = 0.82;

    Mat frame;
    cvtColor(in_frame, frame, CV_RGBA2BGR);

    // Store BGR image for template matching
    // Mat colorImage = frame.clone();

    // Grab only RED channel
    Mat redChannel;
    extractChannel(frame, redChannel, 2);

    // Top hat morph transform
    Mat element = getStructuringElement(MORPH_RECT, Size(11, 11));
    /// Apply the specified morphology operation
    morphologyEx(redChannel, redChannel, MORPH_TOPHAT, element);

    // Otsu's thresholding (Omitted on purpose)
    // Possibly not needed

    // Grab relevant frame part
    Mat redChannelRelevant = redChannel.rowRange(0, frame.rows/2);

    // Find blobs in this frame (red channel relevant part)
    std::vector<KeyPoint> blobs;
    mDetector->detect(redChannelRelevant, blobs);

    // Check if filling this keypoint creates non-circular blobs
    // If it does remove this blob, else keep it and expand that keypoint to new shape
    // TODO: Implement this part if needed

    // Confirm blobs are traffic lights (template matching)

    std::vector<KeyPoint>::iterator blob = blobs.begin();
    while(blob != blobs.end()) {
        Mat imagePart = getTrafficLightFromKeypoint(frame, *blob);

        if (templateMatchScore(imagePart, *mTemplate1) >=
                TEMPLATE_MATCH_THRESHOLD) {
            mRedLightDetected = true;
            break;
        } else if (templateMatchScore(imagePart, *mTemplate2) >=
                        TEMPLATE_MATCH_THRESHOLD) {
            mRedLightDetected = true;
            break;
        } else if (templateMatchScore(imagePart, *mTemplate3) >=
                        TEMPLATE_MATCH_THRESHOLD) {
            mRedLightDetected = true;
            break;
        }
        ++blob;
    }
}

bool DrivingAssistant::isRedLightDetected() {
    return mRedLightDetected;
}

// Methods for LANE DETECTION

bool DrivingAssistant::isLaneDeparted() {
    return mLaneDepartureDetected;
}

void DrivingAssistant::detectLanesInFrame(Mat &frame) {
    cv::Vec2f intersections;
    cv::Vec2f slopes;
    bool lanesDetected = laneIntersections(frame, intersections, slopes);
    if (lanesDetected)  {
        mLaneIntersectionLeft = 0.8 * mLaneIntersectionLeft + 0.2 * intersections[0];
        mLaneIntersectionRight = 0.8 * mLaneIntersectionRight + 0.2 * intersections[1];
    }

    if (mLaneIntersectionLeft > frame.cols * 0.25 || mLaneIntersectionRight < frame.cols * 0.75) {
        mLaneDepartureDetected = true;
    }
    else {
        mLaneDepartureDetected = false;
    }

    circle(frame, cv::Point(mLaneIntersectionLeft, frame.rows), 5, cv::Scalar(255, 0, 0), 2);
    circle(frame, cv::Point(mLaneIntersectionRight, frame.rows), 5, cv::Scalar(255, 0, 0), 2);
}

bool DrivingAssistant::detectLaneIntersection(Mat &region, Point region_offset, Size image_size, Vec2f angle_range, float &intersection, float &slope)
{
    Mat hls;
    cvtColor(region, hls, COLOR_BGR2HLS);

    Mat gray_region;
    extractChannel(hls, gray_region, 1);

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

