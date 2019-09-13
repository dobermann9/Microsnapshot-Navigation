//Microsnapshot Navigation
//Author: Tristan Baumann

#include "lemon/list_graph.h"
#include "lemon/dijkstra.h"
#include "lemon/adaptors.h"
#include "lemon/lgf_writer.h"
#include "lemon/lgf_reader.h"

#include "udp_server.h"
#include "udp_client.h"
#include "sn2_functions.h"

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "boost/asio.hpp"

#include <iostream>
#include <random>
#include <chrono>

//for pi usage
#define _USE_MATH_DEFINES
#include <math.h>

typedef unsigned char uchar;

int main()
{

	//variable declarations ====================================

	//number of SURF features considered in each frame
	int kNumSurfFeatures = 30; //TODO: const
	//descriptor distance under which SURF features are considered to be the same
	const float kSimilarityThreshold = 0.125;
	//the number of surrounding features two features have to share to be considered the same
	const int kNeighborThreshold = 3; //3
	//the maximum number of edges that can be added to a node per update step
	const int kEdgesPerNode = 2;
	//the maximum number of edges a node can have in total
	const int kMaxEdges = 100;
	//dijkstra search repetitions for pathfinding
	const int kDijkstraReps = 30;
	//the graph is updated every kGraphUpdateStep frames/loop iterations
	const int kGraphUpdateStep = 12;

	//planar position in the simulation
	std::array<float, 2> current_position = { 0, 0 };
	std::array<float, 2> last_position = { 0, 0 };

	//received image
	cv::Mat frame, frame_copy;

	//SURF detector, bruteforce matcher and containers
	cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(1200, 2, 2, false, true);
	std::vector<cv::KeyPoint> keypoints;
	cv::BFMatcher matcher;
	std::vector<cv::DMatch>matches;
	cv::Mat descriptors;

	//container for all known features
	cv::Mat all_descriptors;

	bool do_once = true;
	bool is_new_graph = true;
	bool graph_can_update = true;

	bool is_pathfollowing = false;
	bool is_pathfinding = false;

	int frame_ctr = 0;

	//reference "compass" direction with random drift generator
	//to simulate integration errors
	float compass_angle = 0;
	std::default_random_engine random_generator;
	std::normal_distribution<float> distribution(0, 0.02);

	//store features (graph nodes) for navigation and pathfinding
	std::vector<lemon::ListDigraph::Node> current_nodes;
	std::vector<lemon::ListDigraph::Node> last_nodes;
	std::vector<lemon::ListDigraph::Node> goal_nodes;
	std::vector<cv::KeyPoint>goal_keypoints;
	cv::Mat goal_descriptors;

	//counter for path search restart after a set amount of frames
	int search_restart_ctr = 0;

	//tracks the last five frames of movement of the agent to detect if it is stuck //TODO does not work
	std::vector<bool> stuck_tracker = { false,false,false,false,false };
	std::vector<float> stuck_position = { 0,0 };
	int stuck_iter = 0;

	//for sending coordinates to the simulation
	//the frist two entries contain the movement coordinates
	//the third entry is for additional commands
	float send_movement[3] = { 0, 0, 0 };

	//contain statistics about a route: Travelled distance and travel time
	std::array<float, 2> travel_update = {};
	float travel_distance = 0;
	std::chrono::time_point<std::chrono::system_clock> timepoint_;

	//map to illustrate graph
	cv::Mat map(800, 1400, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::Mat map_copy(800, 1400, CV_8UC3, cv::Scalar(255, 255, 255));
	//transformation of simulation data to map coordinates
	float map_unit_mp = 4.534;
	float map_offset_x = 534;
	float map_offset_y = 304;

	//for drawing of the dijkstra paths on map and image
	std::vector<lemon::ListDigraph::Arc> draw_paths;
	//for drawing of the movement instructions that are encoded in currently visible nodes
	//as directional arrows on the nodes and a histogram corresponding to panorama angle
	std::vector<lemon::ListDigraph::Node> draw_path_nodes;
	std::vector<cv::KeyPoint> draw_path_keypoints;
	std::array<int, 36>histogram = {};


	cv::namedWindow("image", cv::WINDOW_NORMAL);
	cv::resizeWindow("image", 1280, 240);

	cv::namedWindow("map", cv::WINDOW_NORMAL);
	cv::resizeWindow("map", 1400, 800);

	//key input handler
	char key_press;
	bool is_randomwalking = false;

	//==========================================================

	//Graph declaration=========================================

	lemon::ListDigraph graph;
	lemon::ListDigraph::ArcMap<int> arcmap(graph); //stores edge length
	lemon::ListDigraph::ArcMap<float> anglemap(graph); //stores movement angles
	lemon::ListDigraph::ArcMap<int> anglemap_iter(graph); //stores number of movement angle updates
	lemon::ListDigraph::NodeMap<bool> filtermap(graph); //used to selectively disable graph nodes in pathfinding

	//stores movement instructions for path following
	lemon::ListDigraph::NodeMap<float> instructions(graph);
	lemon::ListDigraph::NodeMap<bool> has_instruction(graph);

	//Stores coordinates to draw a map of the graph
	lemon::ListDigraph::NodeMap<float> xcoord(graph);
	lemon::ListDigraph::NodeMap<float> ycoord(graph);

	//stores the reference direction for angles
	lemon::ListDigraph::NodeMap<float>compassmap(graph,0);
	lemon::ListDigraph::NodeMap<int>compassmap_ctr(graph,0);

	//stores neighbor features
	//i.e., features visible in the same frame
	lemon::ListDigraph neighbors;

	//==========================================================

	//file opening/loading
	//loads map "TuebingenMap.png", if it exists
	if (FileExists("TuebingenMap.png"))
		map = cv::imread("TuebingenMap.png");

	//attempts to load graph, neighbors and descriptors from the respective files
	if (FileExists("graph.lgf") && FileExists("neighbors.lgf") && FileExists("descriptors.bin")) {
		std::cout << "Graph file detected. Attempting to load graph. This may take a few minutes.\n..." << std::endl;
		lemon::digraphReader(graph, "graph.lgf").arcMap("distance", arcmap).arcMap("angle", anglemap).arcMap("angleUpdates", anglemap_iter)
			.nodeMap("xPos", xcoord).nodeMap("yPos", ycoord).nodeMap("Compass", compassmap).nodeMap("compassUpdates", compassmap_ctr).run();
		lemon::digraphReader(neighbors, "neighbors.lgf").run();
		LoadMatBinary("descriptors.bin", all_descriptors);
		std::cout << "Graph loaded successfully." << std::endl;

		is_new_graph = false;

		//draws the loaded graph on the map
		for (lemon::ListDigraph::ArcIt arcit(graph); arcit != lemon::INVALID; ++arcit) {
			DrawTransparentLine(xcoord[graph.source(arcit)] * map_unit_mp + map_offset_x, ycoord[graph.source(arcit)] * -map_unit_mp + map_offset_y,
				xcoord[graph.target(arcit)] * map_unit_mp + map_offset_x, ycoord[graph.target(arcit)] * -map_unit_mp + map_offset_y,
				CV_RGB(0, 0, 255), 0.05, 1, map);
		}
	}

	//launches udp server and client to communicate with simulation
	boost::asio::io_service io_service;
	udp_server server(io_service, 55551);
	udp_client client(io_service, "localhost", "55552");
	std::thread thread_1{ [&io_service]() { io_service.run(); } };

	//start of main program loop
	while (true) {

		//decodes received udp message into positional data and image
		bool has_data = false;
		std::vector<uchar> msg_decoded = server.get_received();
		if (msg_decoded.size() > 17) {
			has_data = true;
			travel_update = current_position;//for path following distance measurement
			memcpy(&current_position[0], &msg_decoded[msg_decoded.size() - 16], 4);
			memcpy(&current_position[1], &msg_decoded[msg_decoded.size() - 8], 4);
			frame = cv::imdecode(cv::Mat(msg_decoded), CV_LOAD_IMAGE_GRAYSCALE);
			frame_copy = frame;
			if (!frame.data) {
				has_data = false;
			}
		}

		//only continues if image input has been received
		if (has_data) {

			if (do_once) {
				last_position = current_position;
				do_once = false;
			}

			//main graph operations
			if (graph_can_update) {

				//detects surf features in frame and computes their descriptors
				surf->detectAndCompute(frame, cv::Mat(), keypoints, descriptors);

				//adds an initial feature to a new graph;
				if (is_new_graph) {
					all_descriptors.push_back(descriptors.row(0).clone());
					lemon::ListDigraph::Node node = graph.addNode();
					lemon::ListDigraph::Node nbr_node = neighbors.addNode();
					xcoord.set(node, current_position[0]);
					ycoord.set(node, current_position[1]);
					compassmap[node] = compass_angle;
					compassmap_ctr[node] = 1;
					is_new_graph = false;
				}

				//reduces the amount of detected features to kNumSurfFeatures
				if (descriptors.rows > kNumSurfFeatures) {
					descriptors.resize(kNumSurfFeatures);
					keypoints.erase(keypoints.begin() + kNumSurfFeatures, keypoints.end());
				}

				//checks if the graph is updated in this frame/loop iteration
				bool do_graph_update = false;
				if (frame_ctr == kGraphUpdateStep) {
					frame_ctr = 0;
					if (!is_pathfollowing)
						do_graph_update = true;
				}
				++frame_ctr;

				//measures travel distance for path statstics
				if (is_pathfollowing && do_graph_update)travel_distance += hypot(travel_update[0] - current_position[0],
					travel_update[1] - current_position[1]);

				//for drawing new nodes
				std::vector<cv::KeyPoint> new_node_keypoints;
				//container for keypoints of currently visible nodes
				//to differentiate them from currently visible features
				std::vector<cv::KeyPoint> current_node_keypoints;

				current_nodes.clear();

				//matches newly detected surf features to all known features to decide
				//if the currently visible features are known or new
				matcher.match(descriptors, all_descriptors, matches);
				for (cv::DMatch& i : matches) {
					bool is_new_node = false;
					if (i.distance < kSimilarityThreshold) {
						//if two features match, neighboring features are also compared
						//counts neighboring features that are the same between the features of match 'i'
						cv::Mat neighbor_comparison(0, 64, 5);
						for (lemon::ListDigraph::OutArcIt arcit(neighbors, neighbors.nodeFromId(i.trainIdx)); arcit != lemon::INVALID; ++arcit) {
							neighbor_comparison.push_back(all_descriptors.row(neighbors.id(neighbors.target(arcit))).clone());
						}
						std::vector<cv::DMatch> neighbor_matches;
						if (neighbor_comparison.size > 0)
							matcher.match(descriptors, neighbor_comparison, neighbor_matches);
						int neighbor_ctr = 0;
						for (cv::DMatch& j : neighbor_matches) {
							if (j.distance < kSimilarityThreshold)
								++neighbor_ctr;
						}
						//if the features share at least kNeighborThreshold neighbors, they are considered the same
						//otherwise, the feature is considered new
						if (neighbor_ctr >= kNeighborThreshold) {
							current_nodes.push_back(graph.nodeFromId(i.trainIdx));
							current_node_keypoints.push_back(keypoints[i.queryIdx]);
							//compass label is updated (if the graph is updated in this frame)
							if (do_graph_update) {
								compassmap_ctr[graph.nodeFromId(i.trainIdx)] += 1;
								float temp;
								if (IterativeCircularMean(compassmap[graph.nodeFromId(i.trainIdx)],
									-M_PI / 640 * keypoints[i.queryIdx].pt.x + M_PI - compass_angle, compassmap_ctr[graph.nodeFromId(i.trainIdx)], temp)) {
									compassmap[graph.nodeFromId(i.trainIdx)] = temp;
								}
							}

						}
						else {
							is_new_node = true;
						}
					}
					else {
						is_new_node = true;
					}

					//creates a new node
					//Bugfix: The check !(i.queryIdx == 0 && i.trainIdx == 0) avoids continuous node addition from the first node if the agent never moves
					if (is_new_node && do_graph_update && !(i.queryIdx == 0 && i.trainIdx == 0)) {

						//adds new nodes to the graph in every graph update step
						all_descriptors.push_back(descriptors.row(i.queryIdx).clone());

						lemon::ListDigraph::Node node = graph.addNode();
						lemon::ListDigraph::Node nbr_node = neighbors.addNode();

						xcoord.set(node, current_position[0]);
						ycoord.set(node, current_position[1]);

						current_nodes.push_back(node);
						current_node_keypoints.push_back(keypoints[i.queryIdx]);
						new_node_keypoints.push_back(keypoints[i.queryIdx]);

						//adds compass label based on feature position in the image and compass direction
						compassmap[node] = -M_PI / 640 * keypoints[i.queryIdx].pt.x + M_PI - compass_angle;
						compassmap_ctr[node] = 1;
					}
				}

				//graph update step
				//adds new edges to the graph
				//and sets the feature neighborhood, i.e., features that are visible at the same time
				if (do_graph_update) {

					//create edge target set that only holds features from the last update step
					//that are currently not visible;
					std::vector<lemon::ListDigraph::Node> diff_nodes;
					for (lemon::ListDigraph::Node& node_i : last_nodes) {
						bool is_gone = true;
						for (lemon::ListDigraph::Node& node_j : current_nodes) {
							if (node_i == node_j) {
								is_gone = false;
								break;
							}
						}
						if (is_gone)
							diff_nodes.push_back(node_i);
					}

					for (lemon::ListDigraph::Node& node_i : current_nodes) {

						//connects neighboring nodes
						for (lemon::ListDigraph::Node& node_j : current_nodes) {
							if (node_i != node_j && GetArc(neighbors, node_i, node_j) == lemon::INVALID) {
								neighbors.addArc(node_i, node_j);
							}
						}

						//adds up to kEdgesPerNode edges between the features of current_nodes and diff_nodes
						//or updates their values
						if (!diff_nodes.empty()) {
							for (int i = 0; i < kEdgesPerNode; ++i) {
								int rnd = rand() % diff_nodes.size();

								//checks if kMaxEdges still allows for edges to be added to the nodes
								if (OutArcCounter(graph, node_i) <= kMaxEdges && OutArcCounter(graph, diff_nodes[rnd]) <= kMaxEdges) {
									//checks if the edge already exists
									lemon::ListDigraph::Arc arc = GetArc(graph, node_i, diff_nodes[rnd]);
									if (arc == lemon::INVALID) {
										//adds a new edge and its opposite edge to the graph
										lemon::ListDigraph::Arc arc_a = graph.addArc(diff_nodes[rnd], node_i);
										lemon::ListDigraph::Arc arc_b = graph.addArc(node_i, diff_nodes[rnd]);

										//labels edges with movement direction
										float angle_a = atan2(current_position[1] - last_position[1],
											current_position[0] - last_position[0]) - compass_angle;
										float angle_b = OppositeAngle(angle_a);
										anglemap.set(arc_a, angle_a);
										anglemap.set(arc_b, angle_b);
										anglemap_iter.set(arc_a, 1);
										anglemap_iter.set(arc_b, 1);
										//set edge length (neccessary for dijkstra search)
										arcmap.set(arc_a, 1);
										arcmap.set(arc_b, 1);
										//draws edge on map
										DrawTransparentLine(xcoord[diff_nodes[rnd]] * map_unit_mp + map_offset_x, ycoord[diff_nodes[rnd]] * -map_unit_mp + map_offset_y,
											xcoord[node_i] * map_unit_mp + map_offset_x, ycoord[node_i] * -map_unit_mp + map_offset_y,
											CV_RGB(0, 0, 255), 0.05, 1, map);
									}
									else {
										//updates values if edge exists already
										//also update the opposite edge
										lemon::ListDigraph::Arc opposite_arc = GetOppositeArc(graph, arc);
										if (opposite_arc != lemon::INVALID) {
											++anglemap_iter[arc];
											++anglemap_iter[opposite_arc];
											float new_angle;
											if (IterativeCircularMean(anglemap[arc], atan2(current_position[1] - last_position[1],
												current_position[0] - last_position[0]) - compass_angle, anglemap_iter[arc], new_angle)) {
												anglemap.set(arc, new_angle);
												anglemap.set(opposite_arc, OppositeAngle(new_angle));
											}
										}
									}
								}
							}
						}
					}
					//last_-values are updated for the next iteration
					last_nodes = current_nodes;
					last_position = current_position;
				}

				//compass reference direction is obtained from currently visible nodes
				//and then additionally modulated by a small amount of random noise
				if (current_nodes.size() > 0) {
					float cmpss_deviation = -M_PI / 640 * current_node_keypoints[0].pt.x + M_PI - compass_angle - compassmap[current_nodes[0]];
					for (int i = 1; i < current_nodes.size(); ++i) {
						IterativeCircularMean(cmpss_deviation, -M_PI / 640 * current_node_keypoints[i].pt.x + M_PI - compass_angle - compassmap[current_nodes[i]], i + 1, cmpss_deviation);
					}
					compass_angle += 0.05f * cmpss_deviation;
					//add random noise to the cardinal direction
					compass_angle += distribution(random_generator);
				}

				//pathfinding and following
				if (is_pathfollowing) {
					if (!current_nodes.empty() && !goal_nodes.empty()) {
						
						//stops other concurrent movement instructions
						//e.g., random walk
						send_movement[2] = 0;

						//pathfinding
						if (is_pathfinding) {
							is_pathfinding = false;
							//prepares graph for pathfinding and following
							for (lemon::ListDigraph::NodeIt nodeit(graph); nodeit != lemon::INVALID; ++nodeit) {
								filtermap[nodeit] = true;//re-enable disabled nodes from previous searches
								has_instruction[nodeit] = false;
							}
							draw_paths.clear();
							//repeatedly Dijkstra-searches the graph kDijkstraReps times 
							//from a random current node to a random goal node
							for (int i = 0; i < kDijkstraReps; ++i) {
								int rand_start = rand() % current_nodes.size();
								int rand_goal = rand() % goal_nodes.size();
								lemon::Path<lemon::ListDigraph> path;
								lemon::FilterNodes<lemon::ListDigraph> filtered_graph(graph, filtermap);
								lemon::dijkstra(filtered_graph, arcmap).path(path)
									.run(current_nodes[rand_start], goal_nodes[rand_goal]);

								//adds movement instructions to nodes along the route
								//and temporarily removes nodes for subsequent searches
								while (!path.empty()) {
									instructions[graph.source(path.front())] = anglemap[path.front()];
									has_instruction[graph.source(path.front())] = true;
									filtermap[graph.source(path.front())] = false;
									draw_paths.push_back(path.front());
									path.eraseFront();
								}
								//re-enables start and goal nodes that were possibly disabled by the path
								for (lemon::ListDigraph::Node& node_i : current_nodes)
									filtermap[node_i] = true;
								for (lemon::ListDigraph::Node& node_i : goal_nodes)
									filtermap[node_i] = true;
							}
						}//end "if(is_pathfinding)"

						//path following
						float xy_movement[2] = { 0, 0 };
						int instruction_ctr = 0;

						draw_path_nodes.clear();
						draw_path_keypoints.clear();
						histogram = {};

						for (int i = 0; i < current_nodes.size(); ++i) {
							if (has_instruction[current_nodes[i]]) {
								++instruction_ctr;
								xy_movement[0] += cos(instructions[current_nodes[i]] + compass_angle);
								xy_movement[1] += sin(instructions[current_nodes[i]] + compass_angle);

								//for drawing of encoded movement information
								draw_path_nodes.push_back(current_nodes[i]);
								draw_path_keypoints.push_back(keypoints[i]);
								//histogram: increase the correct bin
								float histogram_bin = (instructions[current_nodes[i]] + compass_angle) * (18 / M_PI) + 9;
								if (histogram_bin < 0) histogram_bin += 36;
								histogram[(int)histogram_bin] += 1;
							}
						}

						//if the number of visible nodes containing movement instructions is too low
						//over multiple frames, the path search is restarted
						if (instruction_ctr < 2) {
							++search_restart_ctr;
							if (search_restart_ctr == 10) {
								std::cout << "Path search restarted." << std::endl;
								is_pathfinding = true;
							}
						}
						else {
							search_restart_ctr = 0;
							//prepares movement instructions to be sent to the simulation
							//stiffness is added for smoother movement
							xy_movement[0] = xy_movement[0] * 0.65 / instruction_ctr;
							xy_movement[1] = xy_movement[1] * 0.65 / instruction_ctr;
							xy_movement[0] = 0.3 * xy_movement[0] + 0.7 * send_movement[0];
							xy_movement[1] = 0.3 * xy_movement[1] + 0.7 * send_movement[1];
							//establish minimum move distance
							float move_length = hypot(xy_movement[0], xy_movement[1]);
							if (move_length < 0.1) {
								xy_movement[0] *= (0.1 / move_length);
								xy_movement[1] *= (0.1 / move_length);
							}
							send_movement[0] = xy_movement[0];
							send_movement[1] = xy_movement[1];
							send_movement[2] = 0;
						}
					}
					else {
						//when no known features can be detected, the system attempts to reach 
						//known parts of the environment by random walking
						send_movement[2] = 5;
					}

					//TODO: does not work
					//checks if the agent got stuck
					//that is, it did not move above a certain distance over multiple frames
					//for example due to the geometry of the environment or a total cancel of movement instructions
					/*bool is_stuck = false;
					if (hypot(current_position[0] - stuck_position[0], current_position[1] - stuck_position[1]) < 0.05) {
						stuck_tracker[stuck_iter] = true;
					}
					else {
						stuck_tracker[stuck_iter] = false;
					}
					stuck_iter = (stuck_iter + 1) % stuck_tracker.size();
					for (int i = 0; i < stuck_tracker.size(); ++i) {
						if (stuck_tracker[i]) {
							is_stuck = true;
						}
						else {
							is_stuck = false;
							break;
						}
					}
					if (is_stuck) {
						std::cout << "Agent stuck. Restarting path search." << std::endl;
						is_pathfinding = true;
					}
					stuck_position[0] = current_position[0];
					stuck_position[1] = current_position[1];*/


				}//end "if(is_pathfollowing)"

				//goal check
				//checks if the currently visible scene contains goal features
				//also works outside of pathfinding / following
				if (!goal_nodes.empty() && descriptors.rows > 0) {
					std::vector<cv::DMatch> goal_matches;
					goal_keypoints.clear();
					matcher.match(goal_descriptors, descriptors, goal_matches);
					for (cv::DMatch& i : goal_matches) {
						if (i.distance < kSimilarityThreshold)
							goal_keypoints.push_back(keypoints[i.trainIdx]);
					}

					//stops pathfollowing when the goal has been reached
					if (is_pathfollowing && goal_keypoints.size() > 0.35 * goal_nodes.size()) {
						is_pathfollowing = false;
						send_movement[0] = 0;
						send_movement[1] = 0;
						std::cout << "Path following aborted. The goal has been reached." << std::endl;
						kNumSurfFeatures /= 2;
						last_position = current_position;
						last_nodes = current_nodes;
						//TODO:traveled distance does not work
						std::cout << "Traveled distance: " << travel_distance << " | Time: " <<
							std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - timepoint_).count() / 1000 << std::endl;
					}
				}

				//draws feature keypoints into image
				cv::drawKeypoints(frame, keypoints, frame, cv::Scalar(0, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
				cv::drawKeypoints(frame, new_node_keypoints, frame, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
				cv::drawKeypoints(frame, goal_keypoints, frame, cv::Scalar(240, 200, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			}//end "if(graph_can_update)"

			//sends movement instructions to the simulation as a string
			//values are separated by the letters p and q
			client.send(std::to_string(send_movement[0]) + "p" + std::to_string(send_movement[1])
				+ "q" + std::to_string(send_movement[2]));

			//draws paths on the map, and movement instruction on the panorama image
			map.copyTo(map_copy); //allows for temporary changes to the map
			if (is_pathfollowing) {
				for (lemon::ListDigraph::Arc& arc_i : draw_paths) {
					cv::line(map_copy, cv::Point(xcoord[graph.source(arc_i)] * map_unit_mp + map_offset_x, ycoord[graph.source(arc_i)] * -map_unit_mp + map_offset_y),
						cv::Point(xcoord[graph.target(arc_i)] * map_unit_mp + map_offset_x, ycoord[graph.target(arc_i)] * -map_unit_mp + map_offset_y),
						CV_RGB(255, 91, 27), 1, 8, 0);
				}

				cv::drawKeypoints(frame, draw_path_keypoints, frame, CV_RGB(255, 91, 27), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
				for (int i = 0; i < draw_path_nodes.size(); ++i) {
					cv::Point pt = draw_path_keypoints[i].pt;
					pt.x -= 30 * cos(instructions[draw_path_nodes[i]]);
					pt.y -= 30 * sin(instructions[draw_path_nodes[i]]);
					cv::line(frame, draw_path_keypoints[i].pt, pt, CV_RGB(255, 91, 27), 2, 8, 0);
				}
				for (int i = 0; i < histogram.size(); ++i) {
					cv::rectangle(frame, cv::Point(1265 - i * 35, std::fmaxf(240 - histogram[i] * 10, 0.0f)),
						cv::Point(1240 - i * 35, 240), CV_RGB(0, 0, 255), CV_FILLED);
				}
			}
			//draws goal position as small back and green circles on the map
			if (!goal_nodes.empty()) {
				for (lemon::ListDigraph::Node& node_i : goal_nodes) {
					cv::circle(map_copy, cv::Point(xcoord[node_i] * map_unit_mp + map_offset_x, ycoord[node_i] * -map_unit_mp + map_offset_y), 4, CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
					cv::circle(map_copy, cv::Point(xcoord[node_i] * map_unit_mp + map_offset_x, ycoord[node_i] * -map_unit_mp + map_offset_y), 1, CV_RGB(128, 255, 128), CV_FILLED, 8, 0);
				}
			}

			DrawCurrentPosition(current_position[0], current_position[1], map_unit_mp, map_offset_x, map_offset_y, map_copy);

			//draw compass
			cv::line(frame, cv::Point(30, 210), cv::Point(30 + cos(-M_PI / 2) * 20, 210 + sin(-M_PI / 2) * 20), CV_RGB(200, 20, 20), 2, 8, 0);
			cv::line(frame, cv::Point(30, 210), cv::Point(30 - cos(compass_angle - M_PI / 2) * 20, 210 + sinf(compass_angle - M_PI / 2) * 20), CV_RGB(60, 20, 220), 2, 8, 0);
			cv::circle(frame, cv::Point(30, 210), 20, CV_RGB(60, 20, 220), 2, 8, 0);

			cv::imshow("image", frame);
			cv::imshow("map", map_copy);

			//key input handler
			key_press = cv::waitKey(30);
			switch (key_press) {
			case 'o': //set goal to current frame
				goal_nodes = current_nodes;
				goal_descriptors = descriptors;
				//compression for image output
				cv::imwrite("goal_img.jpg", frame_copy, std::vector<int>{CV_IMWRITE_JPEG_QUALITY, 100});
				std::cout << "Goal set. Saved as 'goal_img.jpg'." << std::endl;
				break;

			case 'l': //load 'goal_img.jpg' and set it as goal.
				//requires extraction and matching of SURF features in the image
				if (FileExists("goal_img.jpg")) {
					cv::Mat goal_img = cv::imread("goal_img.jpg");
					surf->detectAndCompute(goal_img, cv::Mat(), keypoints, goal_descriptors);
					if (goal_descriptors.rows > kNumSurfFeatures) {
						goal_descriptors.resize(kNumSurfFeatures);
						keypoints.erase(keypoints.begin() + kNumSurfFeatures, keypoints.end());
					}
					matcher.match(goal_descriptors, all_descriptors, matches);
					goal_nodes.clear();
					for (cv::DMatch& i : matches) {
						if (i.distance < kSimilarityThreshold) {
							goal_nodes.push_back(graph.nodeFromId(i.trainIdx));
						}
					}
					std::cout << "Goal image loaded sucessfully." << std::endl;
				}
				else {
					std::cout << "Could not load 'goal_img.jpg': File not found." << std::endl;
				}
				break;

			case 'p': //start or end pathfinding and following
				//requires a goal
				if (!goal_nodes.empty()) {
					if (!is_pathfollowing) {
						is_pathfollowing = true;
						is_pathfinding = true;
						travel_distance = 0;
						timepoint_ = std::chrono::system_clock::now();
						std::cout << "pathfinding started." << std::endl;
						kNumSurfFeatures *= 2;
					}
					else {
						is_pathfollowing = false;
						send_movement[0] = 0;
						send_movement[1] = 0;
						send_movement[2] = 0;
						client.send(std::to_string(send_movement[0]) + "p" + std::to_string(send_movement[1])
							+ "q" + std::to_string(send_movement[2]));
						std::cout << "pathfinding aborted." << std::endl;
						last_position = current_position;
						last_nodes = current_nodes;
						kNumSurfFeatures /= 2;
					}
				}
				else {
					std::cout << "No goal set. Press o to set the current view as pathfinding goal,\n or l to load goal image." << std::endl;
				}
				break;

			case 'r': //start or end random walk exploration
				if (!is_randomwalking) {
					is_randomwalking = true;
					send_movement[2] = 5; //5 signals random walk start
					std::cout << "random walk exploration started." << std::endl;
				}
				else {
					is_randomwalking = false;
					send_movement[2] = 0;//stop signal
					std::cout << "random walk exploration aborted." << std::endl;
				}
				break;
			case 'm': //stops graph updating
				//this can for example be used to move the agent around without changing the graph
				if (graph_can_update) {
					graph_can_update = false;
					std::cout << "graph updating disabled." << std::endl;
				}
				else {
					graph_can_update = true;
					std::cout << "graph updating enabled." << std::endl;
					last_position = current_position;
					last_nodes.clear();
				}
				break;

			default:;
			}
		}//end "if(has_data)"
		else {
			key_press = cv::waitKey(20);
		}
		//If the 'Esc' key is pressed, the graph is saved and the program ends.
		if (key_press == 27) {
			int node_num = lemon::countNodes(graph);
			int arc_num = lemon::countArcs(graph);

			std::cout << "Saving graph and neighbors as 'graph.lgf' and 'neighbors.lgf'. This may take a few minutes." << std::endl;
			std::cout << "Number of nodes: " << node_num << std::endl;
			std::cout << "Number of arcs: " << arc_num << std::endl;
			std::cout << "..." << std::endl;

			lemon::digraphWriter(graph, "graph.lgf").arcMap("distance", arcmap).arcMap("angle", anglemap).arcMap("angleUpdates", anglemap_iter)
				.nodeMap("xPos", xcoord).nodeMap("yPos", ycoord).nodeMap("Compass", compassmap).nodeMap("compassUpdates", compassmap_ctr).run();
			lemon::digraphWriter(neighbors, "neighbors.lgf").run();

			std::cout << "Saving descriptors as 'descriptors.bin'. This may take a few minutes." << std::endl;
			std::cout << "..." << std::endl;
			SaveMatBinary("descriptors.bin", all_descriptors);
			std::cout << "saving complete. Press 'Esc' to close the program." << std::endl;
			for (;;) {
				key_press = cv::waitKey(100);
				if (key_press == 27)
					break;
			}
			break;
		}
	}
	//exit handler
	server.is_running = false;
	thread_1.join();
	thread_1.~thread();
}
