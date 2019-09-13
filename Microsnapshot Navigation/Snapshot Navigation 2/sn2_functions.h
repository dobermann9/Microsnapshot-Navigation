#pragma once
#include "lemon/list_graph.h"

#include "opencv2/imgproc.hpp"

//TODO: usage commments?
lemon::ListDigraph::OutArcIt GetArc(lemon::ListDigraph& graph, lemon::ListDigraph::Node& source, lemon::ListDigraph::Node& target);
lemon::ListDigraph::OutArcIt GetOppositeArc(lemon::ListDigraph& graph, lemon::ListDigraph::Arc& arc);

int OutArcCounter(lemon::ListDigraph& graph, lemon::ListDigraph::Node& node);

float OppositeAngle(float angle);
bool IterativeCircularMean(float a, float b, int n, float& out);

void DrawTransparentLine(float from_x, float from_y, float to_x, float to_y, cv::Scalar color, float alpha, int thickness, cv::Mat& output);
void DrawCurrentPosition(float x, float y, float tf_unit_multiplier, float tf_x_offset, float tf_y_offset, cv::Mat& map);

bool FileExists(const char* filename);

bool WriteMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);
bool SaveMatBinary(const std::string& filename, const cv::Mat& output);
bool ReadMatBinary(std::ifstream& ifs, cv::Mat& in_mat);
bool LoadMatBinary(const std::string& filename, cv::Mat& output);