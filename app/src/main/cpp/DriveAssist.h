#pragma once
#include <iterator>

#include <opencv2/core/core.hpp>

#include "DriveAssistUtils.h"

using namespace cv;

namespace da
{

	constexpr int frame_rescaled_width = 320;

	// These constants represent the regions of interest
	// for detecting lanes in % of frame width/height
	constexpr double lane_roi_outer_offset = 0.;//0.25;
	constexpr double lane_roi_inner_offset = 0.5;
	constexpr double lane_roi_top_offset = 0.4;//0.6;
	constexpr double lane_roi_bottom_offset = 1.0;//.87;
	constexpr double lane_roi_width = lane_roi_inner_offset - lane_roi_outer_offset;
	constexpr double lane_roi_height = lane_roi_bottom_offset - lane_roi_top_offset;

	constexpr double left_lane_min_angle = 20;
	constexpr double left_lane_max_angle = 70;
	constexpr double right_lane_min_angle = 89+20;
	constexpr double right_lane_max_angle = 179-20;

	constexpr double rad2deg = 57.2957795;
	constexpr double deg2rad = 0.0174532925;

	bool detect_lane_intersection(Mat region, Point region_offset, Size image_size, Vec2f angle_range, float &intersection, float &slope)
	{
		Mat hls;
		cvtColor(region, hls, COLOR_BGR2HLS);

		//imshow("Region" + std::to_string(region_offset.x), region);

		Mat gray_region;
		extractChannel(hls, gray_region, 1);

		//DEBUG_IMG(gray_region);

		Mat thresholded_region; // Unused
		// Perform otsu thresholding to get optimal threshold
		// TODO: figure out how to get value without performing threshold
		double otsu_threshold = threshold(gray_region, thresholded_region, 1, 1, THRESH_OTSU);

		//DEBUG_IMG(thresholded_region);


		Mat region_edges;
		Canny(gray_region, region_edges, otsu_threshold, otsu_threshold * 0.7);

		//imshow("Canny" + std::to_string(region_offset.x), region_edges);

		//DEBUG_IMG(region_edges);

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

		min_angle *= deg2rad;		
		max_angle *= deg2rad;
		
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

		// Debug
		//if (slope < 0)
		//	line(region, Point(intersection_relative_to_region, region_0_to_image_bottom),
		//		Point(intersection_relative_to_region + 500, region_0_to_image_bottom + 500 * slope), Scalar(0, 255, 0), 2);
		//else
		//	line(region, Point(0, region_0_to_image_bottom-intersection_relative_to_region * slope),
		//		Point(intersection_relative_to_region, region_0_to_image_bottom), Scalar(255, 0, 0), 2);
		//imshow("RgnLine" + std::to_string(region_offset.x), region);

		return true;
	}


	/*
	 * Params:
	 *  frame - frame on which the detection is performed
	 *  intersections - output vector of x values where lanes intersect image at bottom
	 * Returns:
	 *  true if both lanes are detected, otherwise false
	 */
	bool lane_intersections(Mat frame, Vec2f &intersections, Vec2f &slopes)
	{
		float ratio;
		Mat frame_small = dautils::resize_keeping_aspect_ratio(frame, frame_rescaled_width, ratio);

		int width = frame_small.cols;
		int height = frame_small.rows;

		Rect left_lane_roi = Rect(
			int(width * lane_roi_outer_offset),
			int(height * lane_roi_top_offset),
			int(width * lane_roi_width),
			int(height * lane_roi_height)
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
		bool detection_success = detect_lane_intersection(
			left_lane,
			left_lane_roi.tl(),
			Size(width, height),
			Vec2f(left_lane_min_angle, left_lane_max_angle),//Vec2f(20.0, 70.0),
			left_intersection_small,
			slopes[0]);

		if (!detection_success)
		{
			return false;
		}

		float right_intersection_small;
		detection_success = detect_lane_intersection(
			right_lane,
			right_lane_roi.tl(),
			Size(width, height),
			Vec2f(right_lane_min_angle, right_lane_max_angle),
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


}



