#ifndef DRIVEASSISTUTILS_H
#define DRIVEASSISTUTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <functional>
#include <vector>

using namespace cv;

namespace dautils 
{
	constexpr double deg2rad = 0.0174532925;

	Mat YUV2RGBA(Mat y, Mat u, Mat v) {
		std::vector<Mat> planes {y, u, v};
		cv::Mat YUV;
		cv::merge(planes, YUV);

		cv::Mat RGBA;
		cv::cvtColor(YUV, RGBA, cv::COLOR_YUV2RGBA_NV21);

		return RGBA;
	}

	/*
		Resize image to specified width, keeping the aspect ratio
		input:
			frame - image to be resized
			new_width - new width to which the image will be resized
			ratio - output argument will be set to ratio between resized and original image
		returns:
			resized frame
	 */
	Mat resize_keeping_aspect_ratio(Mat &frame, int new_width, float &ratio)
	{
		ratio = float(new_width) / float(frame.cols);
	
		int new_height = int(frame.rows * ratio);

		Mat output;
		resize(frame, output, Size(new_width, new_height));
		
		return output;
	}

	/*
		Params:
			matrix - input matrix
			where - condition function to choose rows
		Returns:
			Returns rows from matrix for which the function where(row) returns true
	*/
	Mat select_matrix_rows(Mat &matrix, std::function<bool(Mat)> where)
	{
		Mat output = Mat(0, matrix.cols, matrix.type());

		for (int i = 0; i < matrix.rows; i++)
		{
			Mat row = matrix.row(i);

			if (where(row))
			{
				output.push_back(row);
			}
		}

		return output;
	}
}

#endif