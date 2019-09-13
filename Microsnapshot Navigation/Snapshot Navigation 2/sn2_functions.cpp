#include "sn2_functions.h"

#include "lemon/list_graph.h"

#include "opencv2/imgproc.hpp"

#include <fstream>

//returns a pointer to an arc between source and target if it exists in graph, INVALID otherwise
lemon::ListDigraph::OutArcIt GetArc(lemon::ListDigraph& graph, lemon::ListDigraph::Node& source, lemon::ListDigraph::Node& target) {
	for (lemon::ListDigraph::OutArcIt arcit(graph, source); arcit != lemon::INVALID; ++arcit) {
		if (graph.target(arcit) == target) {
			return arcit;
		}
	}
	return lemon::INVALID;
}

//returns the opposite arc a_ji to arc a_ij, if it exists
//returns INVALID if no opposite arc exists
lemon::ListDigraph::OutArcIt GetOppositeArc(lemon::ListDigraph& graph, lemon::ListDigraph::Arc& arc) {
	for (lemon::ListDigraph::OutArcIt arcit(graph, graph.target(arc)); arcit != lemon::INVALID; ++arcit) {
		if (graph.target(arcit) == graph.source(arc)) {
			return arcit;
		}
	}
	return lemon::INVALID;
}

//returns the number of outgoing arcs of a node
int OutArcCounter(lemon::ListDigraph& graph, lemon::ListDigraph::Node& node) {
	int counter = 0;
	for (lemon::ListDigraph::OutArcIt arcit(graph, node); arcit != lemon::INVALID; ++arcit) {
		++counter;
	}
	return counter;
}

//returns the input angle + pi, normalized into a range between -2pi and +2pi
float OppositeAngle(float angle) {
	double pi = 3.14159265358979323846;
	float res;
	if (angle < 0) {
		res = angle + (float)pi;
	}
	else {
		res = angle - (float)pi;
	}
	return res;
}

//calculates the circular mean between the known angle a and the new angle b at iteration step n
bool IterativeCircularMean(float a, float b, int n, float& out) {
	float x = cos(a) + 1 / (n) * (cos(b) - cos(a));
	float y = sin(a) + 1 / (n) * (sin(b) - sin(a));
	float res = atan2(y, x);
	if ((x == 0 && y == 0) || isnan(res)) {
		return false;
	}
	out = res;
	return true;
}

//draws a transparent cv::line object
void DrawTransparentLine(float from_x, float from_y, float to_x, float to_y, cv::Scalar color, float alpha, int thickness, cv::Mat& output) {
	cv::Rect rect(cv::Point(from_x, from_y), cv::Point(to_x, to_y));
	rect.x -= 1;
	rect.y -= 1;
	rect.height += 2;
	rect.width += 2;
	cv::Mat roi_1 = output(rect);
	cv::Mat roi_2;
	roi_1.copyTo(roi_2);
	cv::line(roi_2, cv::Point(from_x - rect.x, from_y - rect.y), cv::Point(to_x - rect.x, to_y - rect.y), color, thickness, 8, 0);
	cv::addWeighted(roi_2, alpha, roi_1, 1 - (double)alpha, 0, roi_1);
	roi_1.copyTo(output(rect));
}

//draws a cross-shape at point (x,y) on a map
//transformation between point and map coordinates needs to be specified
void DrawCurrentPosition(float x, float y, float tf_unit_multiplier, float tf_x_offset, float tf_y_offset, cv::Mat& map) {
	cv::circle(map, cv::Point(x * tf_unit_multiplier + tf_x_offset, y * -tf_unit_multiplier + tf_y_offset), 4, CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
	cv::circle(map, cv::Point(x * tf_unit_multiplier + tf_x_offset, y * -tf_unit_multiplier + tf_y_offset), 2, CV_RGB(255, 255, 255), CV_FILLED, 8, 0);
	cv::line(map, cv::Point(x * tf_unit_multiplier + tf_x_offset - 10, y * -tf_unit_multiplier + tf_y_offset),
		cv::Point(x * tf_unit_multiplier + tf_x_offset + 10, y * -tf_unit_multiplier + tf_y_offset), CV_RGB(0, 0, 0), 1, 8, 0);
	cv::line(map, cv::Point(x * tf_unit_multiplier + tf_x_offset, y * -tf_unit_multiplier + tf_y_offset - 10),
		cv::Point(x * tf_unit_multiplier + tf_x_offset, y * -tf_unit_multiplier + tf_y_offset + 10), CV_RGB(0, 0, 0), 1, 8, 0);
}

//checks if the specified file exists
bool FileExists(const char* filename)
{
	std::ifstream ifstream_(filename);
	return ifstream_.good();
}

//saving and loading cv::Mat objects
//From BinaryCvMat by takmin: https://github.com/takmin/BinaryCvMat/
bool WriteMatBinary(std::ofstream& ofs, const cv::Mat& out_mat)
{
	if (!ofs.is_open()) {
		return false;
	}
	if (out_mat.empty()) {
		int s = 0;
		ofs.write((const char*)(&s), sizeof(int));
		return true;
	}
	int type = out_mat.type();
	ofs.write((const char*)(&out_mat.rows), sizeof(int));
	ofs.write((const char*)(&out_mat.cols), sizeof(int));
	ofs.write((const char*)(&type), sizeof(int));
	ofs.write((const char*)(out_mat.data), out_mat.elemSize() * out_mat.total());

	return true;
}
bool SaveMatBinary(const std::string& filename, const cv::Mat& output) {
	std::ofstream ofs(filename, std::ios::binary);
	return WriteMatBinary(ofs, output);
}
bool ReadMatBinary(std::ifstream& ifs, cv::Mat& in_mat)
{
	if (!ifs.is_open()) {
		return false;
	}

	int rows, cols, type;
	ifs.read((char*)(&rows), sizeof(int));
	if (rows == 0) {
		return true;
	}
	ifs.read((char*)(&cols), sizeof(int));
	ifs.read((char*)(&type), sizeof(int));

	in_mat.release();
	in_mat.create(rows, cols, type);
	ifs.read((char*)(in_mat.data), in_mat.elemSize() * in_mat.total());

	return true;
}
bool LoadMatBinary(const std::string& filename, cv::Mat& output) {
	std::ifstream ifs(filename, std::ios::binary);
	return ReadMatBinary(ifs, output);
}