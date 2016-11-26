/* pseudo
 * 
 * 	array = openSet, closeSet, f, g, h
 *
 * 	currentNode = f g
 *
 *	function = h, reconstruct_path, 
 *	
 *
 * function A(start, goal){
 * 		openSet = {Start};
 * 		closeSet = {};
 *
 * 		g = start ~ currentNode Cost
 * 		g[start] = 0;
 * 		f = g + h
 * 		f[start] = h
 *
 * 		while(!openSet){
 * 			currentNode = lowest f value node
 * 			if currentNode = goal
 * 				return reconstruct_path(camefrom, currentNode) // End
 *
 * 			openSet.remove(currentNode)
 * 			closeSet.Add(currentNode)
 *
 * 			for(!neighbor){
 * 				if (neighbor == closeSet)
 * 					continue;
 * 				
 * 				t_gScore = g[currentNode] + dist(currentNode, neighbor);
 *
 * 				if (neighbor != openSet)
 * 					openSet.Add(neighbor);
 * 				else if(t_gScore >= g[neighbor])
 * 					continue;
 *			}
 *
 * 			camefrom[neighbor] = currentNode;
 * 			g[neighbor] = t_gScore;
 * 			f[neighbor] = g[neighbor] +h(neighbor, goal);
 *
 * 			// exit while
 * 			return fail
 * 	}
 */

/*
 *	global var ; WIDTH // array WIDTH
 *	x = value / WIDTH , y = value % WIDTH
 *
 *	f[]	; g + h
 *	g[]	; start ~ currentNode Cost
 *	h[] ; currentNode ~ goal Cost
 *  
 *  openSet[]
 *  closeSet[]
 *
 *	camefrom[]	; parent
 *
 *	int currentNode;
 *	int t_gScore;
 */

//#include <stdio.h>
//#include <stdlib.h>

int WIDTH = 0;

typedef struct Node{
	int f = 0;
	int g = 0;
	char state = 0; // none, open, close (0, 1, 2)state
}Node;



/*
 * Astar
 *
 * parameter
 * 	start : Start Node value
 * 	goal : Goal Node value
 * 	arrayWidth : map array WIDTH
 * 	maxCost
 *	
 * local variable
 *	f : g + h
 *	g : start ~ currentNode Cost
 *	h : currentNode ~ goal Cost
 *
 *	openSet
 *	closeSet
 *	camefrom
 *  
 *  int currentNode
 *  int t_gScore
 *
 * global variable
 * 	WIDTH
 */

/*  A* 실행되는 경우
 *
 *	동작을 누를 때
 *	동작 중 장애물을 만날 때(해당 지점을 closeSet으로 지정 후 실행
 *
 *	
 *
 */


set{
	//방크기 입력받고
	
}


loop{
	//신호 기다림
	if(Serial.available()) {
		String direct = Serial.readStringUntil('\n');
		
	}
	
	while(signal){
	

	}

	//자동 수동?
	if(auto){
		//각각에 signal 넣고.
		//Start / Goal / closeSet / 바라볼 위치
			
		//path 따라 움직이고
		moving(start, goal, arrayWidth, maxCost, closeSet);

	}else if(passive){
		//
	}
	
}

void moving(int start, int goal, int arrayWidth, int maxCost, bool* closeSet){
	int * path;
	int current;	// current Index
	//int past, future;  // past, future Index
	int Calculate_tmp_x, Calculate_tmp_y; // temp_mode
	char pathMode, currentMode;

	current = start;
	path = (int *)malloc(sizeof(int)*maxcost);

	path = Astar(start, goal, arrayWidth, maxCost, closeSet);
	
	while(current == goal){
		unsigned int uS = sonar.ping_cm();
		
		Calculate_tmp_x = (path[current]/arrayWidth) - (path[current - 1]/arrayWidth);
		Calculate_tmp_y = (path[current]%arrayWidth) - (path[current - 1]%arrayWidth);
		pastMode = abs(Calculate_tmp_x)*(1-Calculate_tmp_x) + abs(Calculate_tmp_y)*(2 + Calculate_tmp_y);

		if(uS <= 25){
			//끝내고 current를 start로하는 moving
			int detect_obstacle = 0;

			detect_obstacle = ((path[current + 1]/arrayWidth) + (Calculate_tmp_x*2))*arrayWidth
								+ ((path[current + 1] % arrayWidth) + (Calculate_tmp_y*2));
			closeSet[detect_obstacle] = TRUE;
		}
	

		Calculate_tmp_x = (path[current + 1]/arrayWidth) - (path[current]/arrayWidth);
		Calculate_tmp_y = (path[current + 1]%arrayWidth) - (path[current]%arrayWidth);
		currentMode = abs(Calculate_tmp_x)*(1-Calculate_tmp_x) + abs(Calculate_tmp_y)*(2 + Calculate_tmp_y);


		//첫번째 지점이 아니라면 방향 계산 		
		if(current != 1){
			currentMode = (3*pastMode + currentMode) % 4;
		}
		//move
		switch(currentMode){
		case 0:
			forward();
			Serial.println(currentMode);
			break;
		case 1:
			right();
			forward();
			Serial.println(currentMode);
			break;
		case 2:
			backward();
			Serial.println("back");
			break;
		case 3:
			left();
			forward();
			Serial.println("left");
			break;
		default:
			break;
		}

		current++;

	}
	
	
	/* 구현 할 거
	
	unsigned int uS = sonar.ping_cm();

	if(uS <= 25){
		current
	}	
	obstacle detect process
	
	if(obstacle){
		current 를 closeSet으로
		moving (current, goal, arrayWidth, maxCost, closeSet);
	}	
	*/

	return;
}

int* Astar(int start, int goal, int arrayWidth, int maxCost, int* para_closeSet){
	
	/* struct 
	int *f;	// g + h 
	int *g;	// start ~ currentNode Cost
	int *h;	// currentNode ~ goal Cost
	*/

	Node *node;		//node list
	int *openSet;	// openSet list
	int *closeSet;	// closeSet list
	int *camefrom;	// parent list

	int currentNode = start;	
	//int maxOpenNode = 0;
	//int maxCloseNode = 0;
	int t_fScore;	//neighbor_fScore
	int t_gScore;	// neighbor_gScore

	int openSetNum = 0;
	int closeSetNum = 0;

	node = (Node*)malloc(sizeof(Node)*maxCost);
	openSet = (int*)malloc(sizeof(int)*maxCost);
	closeSet = (int*)malloc(sizeof(int)*maxCost);
	camefrom = (int*)malloc(sizeof(int)*maxCost);

	openSet[openSetNum] = start;
	openSetNum++;
	memset(closeSet, 0, sizeof( int ) * maxCost ); 	

	node[start].f = h(start, goal, arrayWidth);
	node[start].g = 0;

	while(openSet[0]){
		//하나씩 집어넣는 거니까 Insertion Sort?
		currentNode = openset[openSetNum];
		
		//End
		if(currentNode == goal)
			return build_Path(camefrom, currentNode);

		//remove currentNode in openSet
		openSet[openSetNum] = 0;
		maxOpenNode--;
		//Add currentNode in closeSet
		closeSet[closeSetNum] = currentNode;
		maxCloseNode++;

		//neighbor
		int neighborIndex = 0;
		for(neighborIndex = 0; neighborIndex < 4; neighborIndex++){
			//case
			switch(neighborIndex){
				case 0:	// left
					neighbor = currentNode - 1;
					break;
				case 1:	// right
					neighbor = currentNode + 1;
					break;
				case 2: // backward
					neighbor = currentNode - arrayWidth;
					break;
				case 3:	// forward
					neighbor = currentNode + arrayWidth;
					break;
				default:
					return fail; //fail
			}
			//
			if(node[neighbor].state == 2) // neighbor is closeSet
				continue;

			t_gScore = node[currentNode].g + h(currentNode, neighbor, arrayWidth); // h function return 1
			t_fScore = t_gScore + h(neighbor, goal, arrayWidth);
			//neighbor is none
			if(node[neighbor].state == 0){
				openSet[maxOpenNode] = neighbor;
				maxOpenNode++;
				node[neighbor].state = 1;	
			}// neighbor`s g Score large
			else if(t_gScore >= node[neighbor].g)
				continue;

			camefrom[maxCameIndex] = currentNode;
			node[neighbor].g = t_gScore;
			node[neighbor].f = t_fScore;
		}
	}
	return 1; //fail	
}

int h(int start, int goal, int arrayWidth){
	int x_dis;
	int y_dis;

	x_dis = abs((start/arrayWidth) - (goal/arrayWidth));
	y_dis = abs((start%arrayWidth) - (goal%arrayWidth));

	return (x_dis + y_dis);
}
