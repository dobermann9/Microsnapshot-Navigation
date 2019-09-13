# Microsnapshot Navigation
### Dual population coding for topological navigation: Combining discrete state-action graphs with distributed spatial knowledge.[ &#91;4&#93; ](#refs)
#### Eberhard Karls Universität Tübingen <br/> Cognitive Neuroscience <br/> Baumann T, Ecke G and Mallot HA.

## Releases

### [1.0.0] - Initial release - 13.09.2019
#### Added
- README containing Program and usage description.
- *\Compiled Executables* folder containing compiled program executables to be used as-is.
- *\Microsnapshot Navigation* folder containing the main Microsnapshot Navigation project files
- *\Unity Project Files* folder containing the Unity project files.

##  Contents

1. [ Description ](#desc)
2. [ Precompiled Files ](#prec)  
  2.1. [ Usage ](#21)  
  2.2. [ Controls: Snapshot Navigation 2.exe ](#22)  
  2.3. [ Controls: Virtual Tübingen - Snapshot Navigation.exe ](#23)  
  2.4. [ Graph files ](#24)
3. [ Editing the code ](#edit)  
  3.1. [ Main program - Micronsnapshot Navigation ](#31)  
  &nbsp;&nbsp;&nbsp;3.1.1. [ The main() method - Overview ](#311)  
  &nbsp;&nbsp;&nbsp;3.1.2. [ The main() method - Parameter Tuning ](#312)  
  &nbsp;&nbsp;&nbsp;3.1.3. [ sn2_functions.cpp: Helper functions ](#313)  
  &nbsp;&nbsp;&nbsp;3.1.4. [ udp_client.cpp and udp_server.cpp ](#314)  
  3.2. [ The Unity project files ](#32)  
  3.3. [ Communication between the programs ](#33)
4. [ Known issues ](#issues)
5. [ References](#refs)

<a name="desc"></a>
## 1. Description

This repository contains code, project files and compiled executables used for *Dual population coding for topological navigation: Combining discrete state-action graphs with distributed spatial knowledge.*[ &#91;4&#93; ](#refs)
>**Abstract:**
> Topological schemes for navigation from visual snapshots have been based on graphs of panoramic images and action links allowing the transition from one snapshot point to the next; see, for example, Cartwright and Collett[ &#91;1&#93; ](#refs) or Franz et al.[ &#91;2&#93; ](#refs). These algorithms can only work if at each step a unique snapshot is recognized to which a motion decision is associated. Here, we present a population coding approach in which place is encoded by a population of recognized “micro-snapshots” (i.e. features), each with an associated action. Robot motion is then computed by a voting scheme over all activated associations. The algorithm was tested in a large virtual environment (Virtual Tübingen[ &#91;3&#93; ](#refs)) and shows biologically plausible navigational abilities.

This project, in the following shortened to **Microsnapshot Navigation**, is a navigation algorithm which is able to navigate large environments by utilizing only monocular image information and local directional movements. This is realized in two steps: First, the environment is explored (e.g., by random walking) and a graph of small image patches (i.e., nodes are [SURF features], the titular microsnapshots) linked by directional movement information is created. Then, the algorithm may navigate to any known position within the explored environment by combining multiple shortest paths in the graph in a voting scheme.

The project is split in two parts: The main program, i.e., the navigation algorithm, can be found in the subfolder *\Microsnapshot Navigation*. It is written in C++ and comes with a Microsoft Visual Studio solution file. The *\Unity Project Files* subfolder contains an empty virtual environment with panoramic camera agent to test out the algorithm. The files are a Unity project designed to be used with the [Unity](https://unity.com/) editor. The virtual environment within the project is empty due to file size restrictions.

Both projects are also available as precompiled *.exe* files in their respective subfolders under *\Compiled Executables*. The files were generated for 64-bit Windows 10. In contrast to the project files, the virtual environment in the precompiled Unity files (*Virtual Tübingen - Snapshot Navigation.exe*) is not empty but shows a virtual town based on the town center of Tübingen, Germany.

<a name="prec"></a>
## 2. Precompiled files

<a name="21"></a>
### 2.1. Usage

When the main program, named *Snapshot Navigation 2.exe*, is started, three separate windows will pop up: The *console*, displaying command line output, the *image* window displaying a 360° view of the environment with SURF features (black circles), and the *map* window displaying a map of the Virtual Tübingen environment (based on the image *TuebingenMap.png*). Initially, the windows will display nothing, until image and positional information arrive from the Unity program. When a microsnapshot graph file (*graph.lgf*) is in the same folder, the program will attempt to load it which may take a few minutes, depending on graph size. The *map* window will display the current graph as a set of blue lines corresponding to the graph's edges. It also displays the current position (black cross), and the goal (green dots) and set of paths the agent follows during navigation (orange lines).

The unity program, *Virtual Tübingen - Snapshot Navigation.exe*, will initially display a settings window which allows the user to change display settings and to rebind keys. **IMPORTANT: The program was only tested extensively with a resolution of 1280x720px and Graphics quality: Beautiful. Other settings are not guaranteed to work.** After confirming settings with the *Play!* button, the program will show three views of a virtual world: A 360° first person panorama view which is sent to the main program for processing, and top-down and third person views showing the agent's position (black cylinder) within the virtual world. The Unity simulation and the main program communicate over the UDP ports 55551 and 55552 which may potentially need to be forwarded.

<a name="22"></a>
### 2.2. Controls: Snapshot Navigation 2.exe

The main program, *Snapshot Navigation 2.exe*, may be controlled by various key inputs which are described in the following. Note that either the *map* or the *image* window need to be active for the key inputs to work. Also, due to a bug, the inputs will occasionally be ignored and may need to be pressed multiple times. The console window will display confirmation prompts if key inputs have been accepted.

- ````o```` Sets the current frame (i.e., the currently visible graph nodes) as the navigation goal and saves it as *goal_img.jpg*. This will overwriting existing *goal_img.jpg* files.
- ````l```` Attempts to load *goal_img.jpg* as the navigational goal. If the image file exists, its features are extracted and matched to the known features within the graph. All Matches are then the goal nodes for navigation.
- ````p```` Controls navigation: When the key is pressed for the first time, navigation starts by finding shortest routes from the currently visible nodes to the goal nodes, and following their combined movement instructions. Note that the program may initially freeze for a few seconds since pathfinding is quite computationally expensive. Navigation ends if the goal is reached or if the key is pressed again.
- ````r```` Attempts to start or end random walk for exploration. The exact random walk behavior is defined in the Unity simulation, not in the main algorithm.
- ````m```` Disables and re-enables graph updating, i.e., the addition and update of graph nodes. This is useful if the user wants to reposition the agent without updating the graph, for example after setting a goal with ````o````. As a visual cue, if graph updating is disabled, the compass will turn gray and no black feature circles are visible in the *image* window.
- ````Esc```` Ends the program and saves the current graph as *graph.lgf*, *neighbors.lgf* and *descriptors.bin*, overwriting previous files. Saving may take a while, depending on graph size. Afterwards, a *saving complete* prompt will appear and the program may be ended safely by another ````Esc```` key press. If the user wishes to end the program without saving the graph, the program should be closed by alternate means (e.g., simply closing the console window).

Exploration behavior, that is, the addition of new nodes to the microsnapshot graph, is always active, except during navigation (````p````) or if it is disabled (````m````). However, new nodes are only added if the image changes sufficiently, so nothing will happen if the environment is static and the agent does not move. The addition of new nodes is marked by a red flash of the corresponding feature. Currently, the virtual environment (*Virtual Tübingen - Snapshot Navigation.exe*) may be explored in two ways, random walk or manual guidance:

<a name="23"></a>
### 2.3. Controls: Virtual Tübingen - Snapshot Navigation.exe

When the unity program is active, the agent can be manually moved around the virtual environment with the ````w a s d```` or the arrow keys ````↑ ← ↓ →````. The movement is performed relative to the 360° panorama image center (the direction of ````w```` or ````↑````) or the top-down view. The agent never rotates. Additionally, movement may be sped up by holding down the ````shift```` key. It is recommended to disable graph updating during fast movements (by pressing ````m```` with an active main program window) since they are likely to create faulty graph connections.

<a name="24"></a>
### 2.4. Graph files

When the graph is saved by exiting the main programm with the ````Esc```` key, three separate files are created, *graph.lgf*, *neighbors.lgf* and *descriptors.bin*:
- *graph.lgf* contains the basic structure and labels of the microsnapshot graph, delimited by empty space " ". It consists of two disproportionate parts, the graph nodes (`@nodes`) and edges (`@arcs`). The nodes are defined by an ID (`label`) and have with a position within the virtual environment (`xPos` and `yPos`). This position corresponds to the agent's position when the feature was first detected and added to the graph. This positional information is not used for navigation and only serves to draw the map. Further labels are the feature's bearing angle (`Compass`) relative to the compass direction and an iterator to control the mean bearing angle (`compassUpdates`).

The graph edges consist of a pair of node IDs, the start and end nodes, and also have an internal edge ID (`label`). They further have an edge length or weight value (`distance`) which is currently 1 for all nodes (but is required for graph search), a movement angle for navigation (`angle`) and an iterator to control the mean movement angle (`angleUpdates`). Note that two successive edges always form a opposite pair.
- *neighbors.lgf* is basically a list of neighboring nodes for each node in the graph. Two nodes are considered "neighbors" if they are visible at the same time. The list is structured as a graph file; if an edge between two nodes *a* and *b* exists under `@arcs`, *b* is a neighbor of *a*.
- *descriptors.bin* stores the 64-dimensional description vectors of the SURF features corresponding to each node. Each descriptor consists of 64 floating point numbers (4 byte), i.e., has a size of 265 bytes. The file is binary encoded.

<a name="edit"></a>
## 3. Editing the code

<a name="31"></a>
### 3.1. Main program - Micronsnapshot Navigation

The microsnapshot navigation project files contain four *.cpp* C++ code files and accompanying headers. *Snapshot Navigation 2.cpp* contains the main body, i.e., the program's `main()` method. *sn2_functions.cpp* contains various additional functions such as calculations or file reading and writing. Finally, *udp_client.cpp* and *udp_server.cpp* contain functions related to the UDP data transfer between the main program and the Unity simulation. The program relies on three external libraries: [OpenCV 3.4](https://opencv.org/opencv-3-4/) for image processing, the [LEMON](https://lemon.cs.elte.hu/trac/lemon) graph library for graph structure and operations and [Boost.Asio](https://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio.html) for UDP data transfer. Note that the graph terminology of the LEMON graph library slightly differs from the one used here: Edges are called arcs internally.

<a name="311"></a>
#### 3.1.1. The main() method - Overview

![Flowchart](flowchart.png?raw=true "Code Flowchart")
Simplified flowchart depicting the structure of the main microsnapshot navigation program (i.e., the `main()` method). The overall program runs in a large loop and consists of two mostly separate parts: Graph creation and navigation.
- **Graph creation:** Graph creation relies on exploratory movement in the virtual environment providing the input frames. When a recent frame arrives, SURF features are extracted and matched with the descriptors of known nodes to form the set of currently visible nodes (yellow). Then, new edges are added between some of the nodes visible in the preceding frame and the current nodes and labeled with the current movement direction (provided by the virtual environment). Finally, the compass direction and respective feature bearings are updated. This process will continue for as long as the input frames change, or until the user stops graph creation and starts navigation.
- **Navigation:** (Starts at ````425  if (is_pathfollowing) {````) Navigation is again divided into two distinct parts: Pathfinding and path following. The process starts with pathfinding, which requires the currently visible nodes and a user-selected goal location in form of an image of a known location. Known nodes are extracted from the goal image and the shortest paths from random current nodes to goal nodes are found. Then, the movement instructions of the path edges are labeled on the path nodes to enable path following. Path following relies on labeled path nodes: When labeled path nodes are encountered, their movement instructions are averaged and will ideally lead the agent to a location where more labeled path nodes can be encountered, until the goal is reached. If no labeled nodes are detected for a few successive frames, the algorithm has lost the way and path search is restarted. The user may also always stop pathfinding to continue graph creation at any point.

<a name="312"></a>
#### 3.1.2. The main() method - Parameter Tuning

The performance of the program highly depends on various parameters such as the number of SURF features per frame, the feature similarity threshold, the amount of edges per graph node and the frequency of graph updates. The default parameters provided the best results among a set of arbitrary selected alternatives but are certainly not optimized. The parameters may be changed in the main *Snapshot Navigation.cpp* file at the beginning of the `main()` method. The default values are:
```
33  //number of SURF features considered in each frame
34  int kNumSurfFeatures = 30;
35  //descriptor distance under which SURF features are considered to be the same
36  const float kSimilarityThreshold = 0.125;
37  //the number of surrounding features two features have to share to be considered the same
38  const int kNeighborThreshold = 3;
39  //the maximum number of edges that can be added to a node per update step
40  const int kEdgesPerNode = 2;
41  //the maximum number of edges a node can have in total
42  const int kMaxEdges = 100;
43  //dijkstra search repetitions for pathfinding
44  const int kDijkstraReps = 30;
45  //the graph is updated every kGraphUpdateStep frames/loop iterations
56  const int kGraphUpdateStep = 12;
```
For SURF feature detection, it may also be of interest to change the SURF detector itself: Possible options include the Hessian detection threshold, number of octaves (scale levels), number of layers per octave The SURF detector is defined, whether to use extended descriptors (128 dimensions) and whether to consider features upright. The default parameters are (in this order):
```
56  cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(1200, 2, 2, false, true);
```

<a name="313"></a>
#### 3.1.3. sn2_functions.cpp: Helper functions

The *sn2_functions.cpp* file contains various helper functions for the main program:

Graph operations
- `lemon::ListDigraph::OutArcIt GetArc(lemon::ListDigraph& graph, lemon::ListDigraph::Node& source, lemon::ListDigraph::Node& target)` Determines if the edge between two nodes *source* and *target* exists in the graph and returns a pointer to it if it does.
- `lemon::ListDigraph::OutArcIt GetOppositeArc(lemon::ListDigraph& graph, lemon::ListDigraph::Arc& arc)` Determines whether the opposite edge of an edge exists, and returns a pointer to it if it does. The opposite edge of an edge from node *a*to *b* is the edge from *b* to *a*.
- `int OutArcCounter(lemon::ListDigraph& graph, lemon::ListDigraph::Node& node)` Returns the number of outgoing edges of the specified *node*. Outgoing edges are edges that start at *node*.

Calculations
- `float OppositeAngle(float angle)` Rotates the input angle by 180°, restricting the result to a range between -360° and +360°. This method is used to label the opposite edge during graph creation.
- `bool IterativeCircularMean(float a, float b, int n, float& out)` Calculates the circular mean of a set of angles iteratively. For this, it requires the iteration step counter *n*. The circular mean corresponds to the smaller angle between two angles and is not defined for exactly 180° difference; In that case, the method returns `false`.

Map drawing
- `void DrawTransparentLine(float from_x, float from_y, float to_x, float to_y, cv::Scalar color, float alpha, int thickness, cv::Mat& output)` Draws a transparent line between two points onto *output*. This method is used for drawing the current graph on the map, with one line corresponding to one edge. The lines are transparent to indicate differences in edge density.
- `void DrawCurrentPosition(float x, float y, float tf_unit_multiplier, float tf_x_offset, float tf_y_offset, cv::Mat& map)` Draws a cross at the specified position on the *map*. It requires the actual position in the simulation (*x*,*y*) and transformation values into map space (*offsets* and *unit_multiplier*) as input. It is used to draw the agent's current position on the map.

File reading and writing
- `bool FileExists(const char* filename)` Checks whether the specified file exists, either by full path or relative to the program's executable location.
- ```
bool WriteMatBinary(std::ofstream& ofs, const cv::Mat& out_mat)
bool SaveMatBinary(const std::string& filename, const cv::Mat& output)
bool ReadMatBinary(std::ifstream& ifs, cv::Mat& in_mat)
bool LoadMatBinary(const std::string& filename, cv::Mat& output)
```
Allow for the saving and loading of OpenCv Mat objects as binary files. This is used to safe the SURF features' descriptors as *Descriptors.bin*. The functions were taken from [BinaryCvMat](https://github.com/takmin/BinaryCvMat/) by takmin.

<a name="314"></a>
#### 3.1.4. udp_client.cpp and udp_server.cpp

Data transmission between the main algorithm and the simulation uses the UDP data transfer protocol. The related functions for the main program use the [Boost.Asio](https://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio.html) library and were adapted from [Boost_asio tutorials](https://www.boost.org/doc/libs/1_36_0/doc/html/boost_asio/tutorial.html).

<a name="32"></a>
### 3.2. The Unity project files

**Important: The precompiled Unity project (*Virtual Tübingen - Snapshot Navigation.exe*) and the loose project files are not the same.** Due to file size issues, the Unity project files do not come with an extensive virtual environment (*"Virtual Tübingen"*). Rather, they only contain a basic empty scene with the virtual agent, i.e., a 360° panoramic camera setup and a black cylinder marking the agent's position in the top-down and third person views.

The Unity project was designed with [Unity 2019.1.0f2](https://unity3d.com/de/get-unity/download/archive). The main simulation program is defined in the *\Assets\Scripts\UDPSend.cs* script file. Following the engine's logic, the file contains specific functions which are called at certain time points (e.g. `public void Start()`, which is called when the program is started, before the first frame is rendered). For more information, see [Order of Execution for Event Functions](https://docs.unity3d.com/Manual/ExecutionOrder.html).

Beyond controlling agent movement and related physics calculations, the program has UDP data transfer functions to communicate with the main program and defines a random walk exploration behavior (`private void RandomWalk()`): The random walk basically consists of a forward movement which changes its direction every 0.5s by up to 20° in either direction. In order to maximize exploration coverage and minimize time spent in corners or moving into walls, it additionally has the ability to reflect off of walls according to the laws of reflection, that is, if the agent hits a wall during random walk, it will move away from it with the mirrored angle, which results in a "bouncing" exploration behavior.

The Unity project files contain a second script file, *\Assets\Scripts\thirdPersonView.cs*. This file controls the behavior of the third person camera in the corresponding view. The code was adapted from the smooth follow Unity standard assets script.

<a name="33"></a>
### 3.3. Communication between the programs

Communication between the main microsnapshot algorithm program and the Unity simulation functions via the UDP data transfer protocol. The programs communicate over ports 55551 and 55552 on the local machine (localhost / 127.0.0.1). In principle, the programs do not depend on one another; rather, they only require input of the correct form: The main program listens on port 55551 and expects input in the form of a 1280x240px JPEG image with additional 16 bytes of positional data. The positional data is assumed to consist of four 4-byte floats describing the camera's position in the virtual environment: (*x, y, z, y-rotation*, with *y* being the height). From these, the program currently only uses the *x* and *z* values which correspond to the agent's global position within the virtual environment. These values are also not directly available to the algorithm; they only serve to draw the map correctly, and the current movement direction relative to the compass direction is derived from them (i.e., the compass dependency is essentially faked).

In return, the main program sends a float triple `{ 0, 0, 0 }` to port 55552. It is transformed into a string delimited by the letters 'p' and 'q' prior to sending. The first two entries of the triple are Unity *x* and *z* coordinates which indicate agent movement (based on sine and cosine of the movement angle), which the Unity program will attempt to perform if the entries are not 0. The third entry of the triple is used to send additional commands. Currently, this value is either a default 0 or the number 5 which causes the Unity program to random walk for as long as the number is sent.

The Unity program listens on port 55552, accordingly, and expects a string which can be decoded into three floats delimited by the letters 'p' and 'q'. The first two entries are assumed to be a vector indicating a movement, i.e., the program will attempt to perform the movement if at least one entry is not 0, with different vector lengths resulting in different speeds. The third float has various associated commands. Currently, the number 5 will cause random walk to start, and the number 6 will cause the agent to teleport back to it's starting position. The latter is currently never called but may for example be used to repeatedly travel along the same route. Note that in principle, it is possible to both follow movement instructions and random walk at the same time, but this is not recommended.

The data transfer loop is finally closed by the Unity program sending a 1280x240px panorama image and 16 bytes of positional data to port 55551. This data is sent regardless if another program listens on port 55551, or not.

Communication ports and IP adress may be changed in
```
186 udp_server server(io_service, 55551);
187 udp_client client(io_service, "localhost", "55552");
```
in the main microsnapshot program's *Snapshot Navigation 2.cpp* and
```
102 IP = "127.0.0.1";
103 port = 55551;

111 port_rec = 55552;
```
in the Unity project's *UDPSend.cs* script file.

The UDP interface is used so that the program providing the simulation may be changed and replaced flexibly, for example by other simulations running on different machines or even real-world camera or robot setups. **Caution:** Note that the microsnapshot navigation algorithm performs global navigation and does not plan local obstacle avoidance. Rather, obstacle avoidance is solved by collision detection in the Unity simulation. When implementing your own solution, especially on a real-world agent, local obstacle avoidance needs to be supplied to avoid crashing.

<a name="issues"></a>
## 4. Known Issues

**Stuck checker not working:** The main Snapshot Navigation program is supposed to have a function that checks whether the agent has become stuck during path following, e.g., due to difficult geometry or because it walked into a corner. The function starts at `530 bool is_stuck = false;` in the *Snapshot Navigation 2.cpp*'s `main()` method but is currently commented out because it does not work in practice; That is, it does not detect the agent getting stuck.

<a name="refs"></a>
## 5. References

<a name="r1"></a>
[1] Cartwright BA, Collet TS. Landmark maps for honeybees. Biological Cybernetics. 1987;57:85 - 93.

<a name="r2"></a>
[2] Franz MO, Schölkopf B, Mallot HA, Bülthoff HH. Learning View Graphs for Robot Navigation. Autonomous Robots. 1998;5:111 - 125.

<a name="r3"></a>
[3] vaan Veen HAHC, Distler HK, Braun SJ, Bülthoff HH. Navigating through a virtual city: Using virtual reality technology to study human action and perception. Future Generation Computer Systems. 1998;14:231 - 242.

<a name="r4"></a>
[4] *to be released.*
