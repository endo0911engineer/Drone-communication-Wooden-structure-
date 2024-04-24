#pragma once
#include "DataBase.h"

void CreateVertex(Vertex* vertex, vector<Edge>* graph, BanedAreas* BA, int TryTimes);

void CreateBanedAreas(BanedAreas* BA);

void CreateVertex(Vertex* vertex, BanedAreas* BA, int TryTimes);

void SortVertex(Vertex* vertex);

void CreateEdge(Vertex* vertex, BanedAreas* BA, vector<Edge>* graph);

void DijkstraMethod(Vertex* vertex, vector<Edge>* graph, Dijkstra* Dij, vector<Edge>* NewGraph);

int DFS(int NodeNumber, vector<Edge>* NewGraph, vector<int>* Root, int Start, int Destination);

void CreateCircleEdge(Vertex* vertex, BanedAreas* BA, vector<Edge>* NewGraph);