#pragma once
#include "DataBase.h"

void CalculateDistanceBetweenTwoPoints(Node* node, double** Distance, double** Distance_x, double** Distance_y, double** Distance_z);

void ConnectTwoPoints(Node* node, double** Distance, int** ConnectedPointFlag, int** NumberOfConnections, RoCo* pRoco);