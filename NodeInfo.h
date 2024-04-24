#pragma once
#include "DataBase.h"

void GenerateNode(Vertex* vertex, vector<Edge>* NewGraph, Node* node, BanedAreas* BA);

void DestinationAndSpeedGeneration(Vertex* vertex, vector<Edge>* NewGraph, Node* node, Cost* pCost, vector<int>* Root, BanedAreas* BA, Dijkstra* Dij);

unsigned int DACost(int NodeNumber, vector<Edge>* MST, Vertex* vetrex, vector<int>* Root, Node* node, Cost* pCost);

void DA(int NodeNumber, Vertex* vertex, vector<Edge>* graph, Dijkstra* Dij, vector<int>* Root, int Start, int Goal);

void Moving(Vertex* vertex, vector<int>* Root, Node* node);

void MoveTo(int NodeNumber, Vertex* vertex, vector<int>* Root, Node* node);