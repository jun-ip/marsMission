#include <iostream> 
#include <fstream>
#include <string>
#include <thread>
#include <sstream>
#include <vector>
#include <queue>
#include <algorithm> 
using namespace std;


int mapSize;

//struct to contain location of point on map
struct location {
	int x, y;
	location(int _x, int _y) : x(_x), y(_y) {}

	bool operator==(const location& other) const {
		return x == other.x && y == other.y;
	}

	bool operator!=(const location& other) const {
		return x != other.x || y != other.y;
	}
};


location previousPosition(-1, -1);//default previousPosition location

vector<location> bedrockBlacklist; //keeping track of bedrock

//for f cost, g cost, and h cost
struct Node {
	location pos;
	int f, g, h;

	Node(location _pos, int _g, int _h) : pos(_pos), g(_g), h(_h), f(_g + _h) {}

	bool operator>(const Node& other) const {
		return f > other.f;
	}
};

//using Manhattan heuristic, less turns and stuff
int calculateHeuristic(location current, location target) {
	return abs(current.x - target.x) + abs(current.y - target.y);
}

//checking validity of neighbor nodes, target nodes, nodes in the way of path.
bool isValid(int x, int y, vector<vector<char>>& grid) {
	if (x < 0 || x >= grid.size() || y < 0 || y >= grid[0].size()) {
		return false;
	}

	for (auto& blacklisted : bedrockBlacklist) {
		if (location(x, y) == blacklisted) {
			return false;
		}
	}

	return grid[y][x] != 'B' && grid[y][x] != 'F';
}

//checking neighbors for next move, also preventing from going to previous position to prevent oscillation
vector<location> getNeighbors(location current, vector<vector<char>>& grid, int id) {
	vector<location> neighbors;
	neighbors.emplace_back(current.x - 1, current.y);
	neighbors.emplace_back(current.x + 1, current.y);
	neighbors.emplace_back(current.x, current.y - 1);
	neighbors.emplace_back(current.x, current.y + 1);


	vector<location> validNeighbors;
	for (auto& neighbor : neighbors) {
		if (isValid(neighbor.x, neighbor.y, grid) && !(neighbor == previousPosition)) {
			validNeighbors.push_back(neighbor);
		}
	}

	return validNeighbors;
}

//A* pathfinding
vector<location> pathFind(location start, location target, vector<vector<char>>& grid, int id) {

	priority_queue<Node, vector<Node>, greater<Node>> openSet;
	vector<vector<bool>> closedSet(mapSize, vector<bool>(mapSize, false));
	vector<vector<Node>> nodeInfo(mapSize, vector<Node>(mapSize, Node(location(-1, -1), -1, -1)));

	openSet.push(Node(start, 0, calculateHeuristic(start, target)));

	while (!openSet.empty()) {

		Node current = openSet.top();
		openSet.pop();

		if (current.pos.x == target.x && current.pos.y == target.y) {
			vector<location> path;
			while (!(current.pos.x == start.x && current.pos.y == start.y)) {
				path.push_back(current.pos);
				current = nodeInfo[current.pos.x][current.pos.y];
			}
			path.push_back(start);
			reverse(path.begin(), path.end());
			return path;
		}

		closedSet[current.pos.x][current.pos.y] = true;

		vector<location> neighbors = getNeighbors(current.pos, grid, id);

		for (const location& neighbor : neighbors) {
			if (isValid(neighbor.x, neighbor.y, grid) && !closedSet[neighbor.x][neighbor.y]) {
				int testG = current.g + 1;

				if (testG < nodeInfo[neighbor.x][neighbor.y].g || nodeInfo[neighbor.x][neighbor.y].g == -1) {
					openSet.push(Node(neighbor, testG, calculateHeuristic(neighbor, target)));
					nodeInfo[neighbor.x][neighbor.y] = Node(current.pos, testG, calculateHeuristic(neighbor, target));
				}
			}
		}
	}

	//empty list
	return vector<location>();
}


//unused
void printGrid(vector<vector<char>>& grid) {
	for (auto row : grid) {
		for (char cell : row) {
			std::cout << cell << ' ';
		}
		std::cout << '\n';
	}
}

//bedrock tracking
void printBedrockCoordinates(vector<vector<char>>& grid) {
	for (int i = 0; i < grid.size(); ++i) {
		for (int j = 0; j < grid[i].size(); ++j) {
			if (grid[i][j] == 'B') {
				cout << "Bedrock at: (" << j << ", " << i << ")" << endl;
				bedrockBlacklist.push_back(location(j, i));//add bedrock to blacklist
			}
		}
	}
}

//checking for resources in vision, default 5x5.
bool hasResourceVision(location& current, vector<vector<char>>& grid, int visionWidth, int visionHeight) {
	for (int dx = -visionWidth / 2; dx <= visionWidth / 2; ++dx) {
		for (int dy = -visionHeight / 2; dy <= visionHeight / 2; ++dy) {
			int x = current.x + dx;
			int y = current.y + dy;

			if (x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size()) {
				if (grid[y][x] == 'C' || grid[y][x] == 'D') {//iron and osmium
					return true;
				}
			}
		}
	}

	return false;
}

//returns location of resource in vision.
location findResourceVision(location& current, vector<vector<char>>& grid, int visionWidth, int visionHeight) {
	for (int dx = -visionWidth / 2; dx <= visionWidth / 2; ++dx) {
		for (int dy = -visionHeight / 2; dy <= visionHeight / 2; ++dy) {
			int x = current.x + dx;
			int y = current.y + dy;

			if (x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size()) {
				if (grid[y][x] == 'C' || grid[y][x] == 'D') {
					return location(x, y);
				}
			}
		}
	}

	return location(-1, -1);
}

//checks for if nextnext move will be a turn for efficient turns.
bool isTurn(location current, location next, location nextNext) {
	if (next.x == current.x) {
		if (next.y == nextNext.y && next.x != nextNext.x) {
			return true;
		}
	}
	else if (next.y == current.y) {
		if (next.x == nextNext.x && next.y != nextNext.y) {
			return true;
		}
	}
	return false;
}

//mining in nextnext move direction for efficient turning.
string mineInDirection(location& current, location& nextMove, location& nextNextMove) {

	//mining in nextNextMove direction
	if (isTurn(current, nextMove, nextNextMove)) {
		if (nextNextMove.x < nextMove.x) {
			return " M L";
		}
		else if (nextNextMove.x > nextMove.x) {
			return " M R";
		}
		else if (nextNextMove.y < nextMove.y) {
			return " M U";
		}
		else if (nextNextMove.y > nextMove.y) {
			return " M D";
		}
	}
	return "";
}

//checking player in vision
bool hasPlayer(location& current, vector<vector<char>>& grid, int visionWidth, int visionHeight, int id) {
	for (int dx = -visionWidth / 2; dx <= visionWidth / 2; ++dx) {
		for (int dy = -visionHeight / 2; dy <= visionHeight / 2; ++dy) {
			int x = current.x + dx;
			int y = current.y + dy;

			if (x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size()) {
				if ((grid[y][x] - '0') != id && (grid[y][x] == '0' || grid[y][x] == '1' || grid[y][x] == '2' || grid[y][x] == '3' || grid[y][x] == '4')) {
					cout << "FOUND PLAYER " << grid[y][x] << endl;
					return true;
				}
			}
		}
	}

	return false;
}

//returning location of player in vision.
location findPlayer(location& current, vector<vector<char>>& grid, int visionWidth, int visionHeight, int id) {
	for (int dx = -visionWidth / 2; dx <= visionWidth / 2; ++dx) {
		for (int dy = -visionHeight / 2; dy <= visionHeight / 2; ++dy) {
			int x = current.x + dx;
			int y = current.y + dy;

			if (x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size()) {
				if ((grid[y][x] - '0') != id && (grid[y][x] == '0' || grid[y][x] == '1' || grid[y][x] == '2' || grid[y][x] == '3' || grid[y][x] == '4')) {
					return location(x, y);
				}
			}
		}
	}

	return location(-1, -1);
}

//attacking in direction of player
string attackInDirection(location& current, location player) {

		if (player.x < current.x) {
			return " A L";
		}
		else if (player.x > current.x) {
			return " A R";
		}
		else if (player.y < current.y) {
			return " A U";
		}
		else if (player.y > current.y) {
			return " A D";
		}
	return "";
}

int main()
{
	int id = 0;
	std::cout << "enter id: ";
	std::cin >> id;

	int round = 0;


	int maxOsmium = 0;

	//targetlist of all nodes to meet, get heuristic of all and pathfind to closest
	vector<location> targetList = {
	{21, 7},
	{13, 9},
	{18, 15},
	{36, 11},
	{46, 8},
	{48, 17},
	{77, 12},
	{87, 16},
	{68, 28},
	{39, 27},
	{34, 31},
	{14, 27},
	{3, 32},
	{16, 40},
	{48, 41},
	{66, 38},
	{67, 53},
	{75, 55},
	{48, 54},
	{31, 47},
	{28, 49},
	{11, 58},
	{9, 61},
	{26, 66},
	{52, 60},
	{76, 64},
	{72, 79},
	{63, 72},
	{50, 75},
	{35, 72},
	{22, 78},
	{13, 79},

	};

	vector<location> targetList2 = {
	{56, 13},
	{46, 12},
	{44, 4},
	{38, 1},
	{38, 13},
	{27, 16},
	{22, 7},
	{17, 6},
	{3, 3},
	{10, 17},
	{20, 21},
	{3, 23},
	{18, 25},
	{34, 25},
	{47, 29},
	{57, 36},
	{51, 43},
	{45, 39},
	{31, 37},
	{20, 38},
	{11, 38},
	{1, 39},
	{32, 41},
	{40, 43},
	{55, 55},
	{32, 54},
	{23, 50},
	{20, 46},
	{12, 48},
	{10, 50},
	{7, 55},
	};

	vector<location> centerSquare = {
		{44, 44}, //CENTER
		{45, 44}, //MIDDLE RIGHT
		{43, 44}, //MIDDLE LEFT
		{44, 43}, //TOP MIDDLE
		{44, 45}, //BOTTOM MIDDLE
		{43, 43}, //TOP LEFT
		{45, 45}, //BOTTOM RIGHT
		{43, 45}, //BOTTOM LEFT
		{45, 43}, //TOP RIGHT

	};

	vector<location> centerSquare2 = {
		{29, 29}, //CENTER
		{30, 29}, //MIDDLE RIGHT
		{28, 29}, //MIDDLE LEFT
		{29, 28}, //TOP MIDDLE
		{29, 30}, //BOTTOM MIDDLE
		{28, 28}, //TOP LEFT
		{30, 30}, //BOTTOM RIGHT
		{28, 30}, //BOTTOM LEFT
		{30, 28}, //TOP RIGHT

	};

	std::string serverFileName = "game/s" + std::to_string(id) + "_" + std::to_string(round) +
		".txt";
	std::ifstream input(serverFileName);

	//getting base location initially
	string mapLine;
	getline(input, mapLine);
	istringstream iiss(mapLine);
	iiss >> mapSize;
	int loc_number;
	int acid_round = 140;
	location center1(44, 44);
	location center2(29, 29);
	location center(-1, -1);
	location target(-1, -1);
	vector <location>targetListings;
	vector <location>mainSquare;

	if (mapSize == 59) {
		loc_number = 61;
		center = center2;
		target = center2;
		targetListings = targetList2;
		mainSquare = centerSquare2;

	}
	else {
		loc_number = 91;
		center = center1;
		center = center1;
		targetListings = targetList;
		mainSquare = centerSquare;
	}

	int current_line = 1;
	string line;
	while (current_line != loc_number || !input.eof()) {
		current_line++;
		getline(input, line);
		istringstream iss(line);
		if (current_line == loc_number) break;
	}
	//--------------------------------------------------------------

	input.close();

	istringstream iss(line);
	int x, y;
	iss >> x >> y;
	location base(x, y);




	while (true)
	{
		
		
		std::string serverFileName = "game/s" + std::to_string(id) + "_" + std::to_string(round) +
			".txt";

		std::ifstream input(serverFileName);

		if (input)
		{
			cout << loc_number << endl;

			//sleep a little to make sure that the file was written bt the server
			std::this_thread::sleep_for(std::chrono::milliseconds(5));

			//it is our turn to move
			//read the file...
			string firstLine;
			getline(input, firstLine);

			istringstream iiss(firstLine);

			int gridSize;
			iiss >> gridSize;

			mapSize = gridSize;

			vector<vector<char>> grid(gridSize, vector<char>());
			
			int current_line = 1;
			int grid_line = 0;
			string line;

			//getting data from server file, resources, current location, current grid data.
			while (current_line != loc_number || !input.eof()){
				current_line++;
				getline(input, line);
				istringstream iss(line);
				if (current_line == loc_number) break;

				char ch;
				while (iss >> ch) {
					grid[grid_line].push_back(ch);
				}
				grid_line += 1;

				
			}
			//current phases, MINING, CENTER, ATTACK
			string phase = "";
			
			//bunch of data just to print and debug
			//--------------------------------------------------------------

			istringstream iss(line);
			int x, y;
			iss >> x >> y;
			location current(x, y);

			//---------------------------------------------------------------

			int health, dig, atk, move, sight;
			bool antenna, battery;

			getline(input, line);
			istringstream ss(line);
			ss >> health >> dig >> atk >> move >> sight >> antenna >> battery;

			//----------------------------------------------------------------

			int stones, iron, osmium;
			
			getline(input, line);
			istringstream s(line);
			s >> stones >> iron >> osmium;

			//---------------------------------------------------------------

			if (osmium > maxOsmium) {
				maxOsmium = osmium;
			}

			//printGrid(grid);
			printBedrockCoordinates(grid);


			input.close();

			//write the response back
			std::string ourFileName = "game/c" + std::to_string(id) + "_" + std::to_string(round) +
				".txt";
			std::ofstream response(ourFileName);
			//..

			//response << "U\n";
			//response << "M U\n";

			// ...

			//calculating closest target and prioritizing those targets using calculateHeuristic.
			//Manhattan distance.
			vector<int> heuristicsTargetList;
			for (const auto& t : targetListings) {
				heuristicsTargetList.push_back(calculateHeuristic(current, t));
			}

			auto minHeuristic = min_element(heuristicsTargetList.begin(), heuristicsTargetList.end());
			int closestTarget = distance(heuristicsTargetList.begin(), minHeuristic);
			//default is 5, i dont act change this i think cuz i dont use sight upgrades
			int vision = 5;

			bool centerPhase = false;
			bool attackPhase = false;
			//check if in center currently
			bool inCenter = find(mainSquare.begin(), mainSquare.end(), current) != mainSquare.end();

			//basic strategy laid out, mine resources before round 140, then round 140 start moving towards center, during which
			//u also mine along the way, as well as attack if running into any players directly and also prioritize
			//getting into the center over attacking first as to try to avoid acid first then attack players as close to center
			if (round < acid_round) {
				if (!battery && (iron >= 1 && osmium >= 1)) {
					target = base;

						if (hasResourceVision(current, grid, vision, vision) && iron <= 9) {
							location resourceLoc = findResourceVision(current, grid, vision, vision);

								target = resourceLoc;
						}
				}
				else if (hasResourceVision(current, grid, vision, vision) && iron <= 9) {
					location resourceLoc = findResourceVision(current, grid, vision, vision);

					target = resourceLoc;
				}
				else {
					target = targetListings[closestTarget];
				}

			}
			else {
				target = center;
				centerPhase = true;

				if (!inCenter) {
					if (hasPlayer(current, grid, vision - 2, vision - 2, id)) {
						attackPhase = true;
						centerPhase = false;
						location playerLoc = findPlayer(current, grid, vision - 2, vision - 2, id);
						target = playerLoc;
					}
					else if (hasResourceVision(current, grid, vision, vision) && iron <= 9) {
						location resourceLoc = findResourceVision(current, grid, vision, vision);

						target = resourceLoc;
					}
				}
				else {
					if (hasPlayer(current, grid, vision, vision, id)) {
						attackPhase = true;
						centerPhase = false;
						location playerLoc = findPlayer(current, grid, vision, vision, id);
						target = playerLoc;
					}
				}
				
				
				
				

			}
			

			//calling pathfinding, calculated every round
			vector<location> path = pathFind(current, target, grid, id);

			//debugging info and also to erase targets once reached. clears path and makes a new one for the next target.
			cout << "Path Size: " << path.size() << endl;
			if (!path.empty()) {
				previousPosition = path[0];
				cout << "Current Position: " << "(" << path[0].x << ", " << path[0].y << ")" << endl;
				if (path.size() > 1) {
					auto targetInList = find(targetListings.begin(), targetListings.end(), target);
					cout << "Next Position: " << "(" << path[1].x << ", " << path[1].y << ")" << endl;
					if (path[1] == target && grid[target.y][target.x] == '.' && target != (center)) {
						if (!targetListings.empty()) {
							targetListings.erase(targetInList);
							path.clear();
							vector<location> path = pathFind(current, target, grid, id);
						}
					}
				}
				
				else {
					cout << "End of path" << endl; //removes target once reached
					if (!targetListings.empty()) {
						targetListings.erase(targetListings.begin() + closestTarget);
					}
				}
			}
			else {
				//cout << "no path" << endl;
				cout << "------------------------------------" << endl;
				cout << "TARGET: " << "(" << target.x << ", " << target.y << ")" << endl;
			}

			//auto inCenter = find(centerSquare.begin(), centerSquare.end(), current);


			//determining next move/action
			if (!path.empty() && path.size() > 1) {

				location nextMove = path[1];
				location nextNextMove(-1, -1);

				if (path.size() > 2) {
					nextNextMove = path[2];
				}
				else {
					nextNextMove = nextMove;
				}


				string moveCommand = "";
				string moveDirection = "";
				string mineDirection = "";
				string attackDirection = "";

				string buyHeal = "";
				string buy = "";

				if ((battery && health <= 4) || (battery && health == 10)) {
					buyHeal = " B H";
				}
				if (battery && atk < 3) {
					buy = " B A";
				}

				if (nextMove.x < current.x) {
					moveDirection = "L";
					mineDirection = " M L";
					attackDirection = " A L";

				}
				else if (nextMove.x > current.x) {
					moveDirection = "R";
					mineDirection = " M R";
					attackDirection = " A R";
				}
				else if (nextMove.y < current.y) {
					moveDirection = "U";
					mineDirection = " M U";
					attackDirection = " A U";

				}
				else if (nextMove.y > current.y) {
					moveDirection = "D";
					mineDirection = " M D";
					attackDirection = " A D";

					
				}
				
				if (grid[nextMove.y][nextMove.x] == '.') {
					if (isTurn(current, nextMove, nextNextMove)) {
						mineDirection = mineInDirection(current, nextMove, nextNextMove);
					}
				}
				
				
				
				

				//debugging stuff for movement
				cout << "Move Direction: " << moveDirection << endl;
				cout << "Mine Direction: " << mineDirection << endl;
				cout << "TURN NEXT: " << isTurn(current, nextMove, nextNextMove) << endl;

				if (attackPhase) {
					moveCommand = moveDirection + attackDirection + buyHeal + buy + "\n";
					phase = "ATTACK";
				}
				else {
					moveCommand = moveDirection + mineDirection + buyHeal + buy + "\n";
					if (centerPhase) {
						phase = "CENTER";
					}
					else {
						phase = "MINING";

					}
				}


				response << moveCommand;

				previousPosition = current;
				//cout << "GRID TEST: " << grid [0][0] << endl;

				//clearing path each round
				path.clear();
			}
			else {
				string moveCommand = "";
				string moveDirection = "";
				string attackDirection = "";

				string buyHeal = "";
				string buy = "";
				//cout << "No valid path found." << endl;
				if (!battery && iron >= 1 && osmium >= 1) {
					response << "B " << "B";
				}
				if (attackPhase && hasPlayer(current, grid, vision, vision, id)) {
					//moveCommand = moveDirection + "" + buyHeal + buy + "\n";

					moveCommand = moveDirection + attackInDirection(current, findPlayer(current, grid, vision, vision, id)) + buyHeal + buy + "\n";
					response << moveCommand;
				}

				previousPosition = location(-1, -1);
				cout << "------------------------------------" << endl;
				cout << "TARGET: " << "(" << target.x << ", " << target.y << ")" << endl;
			}

			// ...


			response.close();

			/* BASE LIST
			*  (79, 55) BOTTOM RIGHT
			*  (44, 81) BOTTOM MIDDLE
			*  (66, 13) BOTTOM LEFT
			*  (21, 13) TOP LEFT
			*  (66, 13) TOP RIGHT
			*/


			cout << "----------------------------------------" << endl;
			cout << "MAP SIZE: " << mapSize << endl;
			cout << "BASE: " << "(" << base.x << ", " << base.y << ")" << endl;
			cout << "ROUND: " << round << endl;
			cout << "PHASE: " << phase << endl;
			cout << "HEALTH: " << health << + " (+" << osmium * 5 << ")" << endl;
			cout << "ATK LVL: " << atk << endl;
			cout << "IRON: " << iron << endl;
			cout << "OSMIUM: " << osmium << endl;
			cout << "MAX OSMIUM: " << maxOsmium << endl;
			cout << "IN CENTER?: " << inCenter << endl;
			cout << "PLAYER NEAR?: " << hasPlayer(current, grid, vision, vision, id) << endl;
			cout << "ATTACKING?: " << attackPhase << endl;
			cout << "----------------------------------------" << endl;

			//increment the round
			round++;
		}
		else
		{
			//waiting
		}
	}
	return 0;
}