#include "NodeInfo.h"

void GenerateNode(Vertex* vertex, vector<Edge>* NewGraph, Node* node, BanedAreas* BA)
{
	int Overlap = 0;

	double Distance_x; //頂点とノードのx方向の距離
	double Distance_y; //頂点とノードののy方向の距離
	double Distance; //頂点とノードのユークリッド距離

	double Min_AbsoluteValue = UINT_MAX;
	int result = 0;

	random_device rd; //オブジェクトの作成

	mt19937 mt(rd()); //擬似乱数の初期シードとして生成したオブジェクトを設定
	uniform_int_distribution<int> GenerateNodeX(0, SERVICE_AREA_X); //生成する頂点のx座標の範囲を設定
	uniform_int_distribution<int> GenerateNodeY(0, SERVICE_AREA_Y); //生成する頂点のy座標の範囲を設定

	for (int i = 0; i < NODE; i++)
	{
		do
		{
			Overlap = 0;

			node[i].Cx = GenerateNodeX(mt);
			node[i].Cy = GenerateNodeY(mt);
			node[i].Cz = 0;

			for (int s = 0; s < NOBA; s++)
			{
				if (((static_cast<int>(BA[s].XA) < node[i].Cx) && (node[i].Cx < static_cast<int>(BA[s].XB))) && ((static_cast<int>(BA[s].YA) < node[i].Cy) && (node[i].Cy < static_cast<int>(BA[s].YB))))
				{
					Overlap = 1;
				}
			}

		} while (Overlap);

		for (int s = 0; s < NUMBER_OF_VERTEX; s++)
		{
			Distance_x = static_cast<double>(fabs(node[i].Cx - vertex[s].Vx));
			Distance_y = static_cast<double>(fabs(node[i].Cy - vertex[s].Vy));

			Distance = hypot(Distance_x, Distance_y); //三平方の定理より2点間の距離を計算

			if (Min_AbsoluteValue > Distance)
			{
				result = s;
				Min_AbsoluteValue = Distance;
			}
		}

		node[i].PS = result; //近くの頂点番号

		Min_AbsoluteValue = UINT_MAX;
	}
}

void DestinationAndSpeedGeneration(Vertex* vertex, vector<Edge>* NewGraph, Node* node, Cost* pCost, vector<int>* Root, BanedAreas* BA, Dijkstra* Dij)
{
	int Overlap = 0;
	double Distance_x; //頂点同士のx方向の距離
	double Distance_y; //頂点同士のy方向の距離
	double Distance; //頂点同士のユークリッド距離

	double Min_AbsoluteValue = UINT_MAX;
	int result = 0;
	double a = 0.0, b = 0.0, res = 0.0;
	double SpeedVector, X, Y, SquareRoot;

	random_device rd; //オブジェクトの作成

	mt19937 mt(rd()); //擬似乱数の初期シードとして生成したオブジェクトを設定
	uniform_int_distribution<int> GenerateDestination_x(0, SERVICE_AREA_X); //ノードの移動先のx座標の範囲を設定
	uniform_int_distribution<int> GenerateDestination_y(0, SERVICE_AREA_Y); //ノードの移動先のy座標の範囲を設定
	uniform_int_distribution<int> range_speed(MIN_SPEED, MAX_SPEED); //ノードの速度の範囲を設定

	for (int i = 0; i < NODE; i++)
	{
		if (node[i].F == 0)
		{
			node[i].SP = range_speed(mt); //速度をランダムに決定
			node[i].F = 1; //ノードを移動開始位置から最も近い頂点に移動の状態に

			while (1) {
				do
				{
					Overlap = 0;

					node[i].Dx = GenerateDestination_x(mt);
					node[i].Dy = GenerateDestination_y(mt);
					node[i].Dz = 0;

					for (int s = 0; s < NOBA; s++)
					{
						if (((static_cast<int>(BA[s].XA) < node[i].Dx) && (node[i].Dx < static_cast<int>(BA[s].XB))) && ((static_cast<int>(BA[s].YA) < node[i].Dy) && (node[i].Dy < static_cast<int>(BA[s].YB))))
						{
							Overlap = 1;
						}
					}

					if ((node[i].Cx == node[i].Dx) && (node[i].Cy == node[i].Dy))
					{
						Overlap = 1;
					}


				} while (Overlap);

				for (int s = 0; s < NUMBER_OF_VERTEX; s++)
				{
					Overlap = 0;

					if (node[i].Dx == vertex[s].Vx)
					{
						if (node[i].Dy < vertex[s].Vy)
						{
							for (int t = 0; t < NOBA; t++)
							{
								if (!((node[i].Dy < BA[t].YA) && (BA[t].YB < vertex[s].Vy)))
								{
									Overlap = 1;
								}
							}
						}
						else
						{
							for (int t = 0; t < NOBA; t++)
							{
								if (!((vertex[s].Vy < BA[t].YA) && (BA[t].YB < node[i].Dy)))
								{
									Overlap = 1;
								}
							}
						}
					}
					else if (node[i].Dy == vertex[s].Vy)
					{
						if (node[i].Dx < vertex[s].Vx)
						{
							for (int t = 0; t < NOBA; t++)
							{
								if (!((node[i].Dx < BA[t].XA) && (BA[t].XB < vertex[s].Vx)))
								{
									Overlap = 1;
								}
							}
						}
						else
						{
							for (int t = 0; t < NOBA; t++)
							{
								if (!((vertex[s].Vx < BA[t].XA) && (BA[t].XB < node[i].Dx)))
								{
									Overlap = 1;
								}
							}
						}
					}
					else
					{
						a = static_cast<double>(vertex[s].Vy - node[i].Dy) / static_cast<double>(vertex[s].Vx - node[i].Dx);
						b = static_cast<double>(node[i].Dy) - a * static_cast<double>(node[i].Dx);

						for (int t = 0; t < NOBA; t++)
						{
							res = a * static_cast<double>(BA[t].XA) + b;

							if ((static_cast<double>(BA[t].YA) < res) && (res < static_cast<double>(BA[t].YB)))
							{
								if (node[i].Dx < vertex[s].Vx)
								{
									if (((node[i].Dx < static_cast<int>(BA[t].XA)) && (static_cast<int>(BA[t].XA) < vertex[s].Vx)))
									{
										Overlap = 1;
									}

								}
								else
								{
									if (((vertex[s].Vx < static_cast<int>(BA[t].XA)) && (static_cast<int>(BA[t].XA) < node[i].Dx)))
									{
										Overlap = 1;
									}
								}
							}
						}

						for (int t = 0; t < NOBA; t++)
						{
							res = a * static_cast<double>(BA[t].XB) + b;

							if ((static_cast<double>(BA[t].YA) < res) && (res < static_cast<double>(BA[t].YB)))
							{
								if (node[i].Dx < vertex[s].Vx)
								{
									if (((node[i].Dx < static_cast<int>(BA[t].XB)) && (static_cast<int>(BA[t].XB) < vertex[s].Vx)))
									{
										Overlap = 1;
									}
								}
								else
								{
									if (((vertex[s].Vx < static_cast<int>(BA[t].XB)) && (static_cast<int>(BA[t].XB) < node[i].Dx)))
									{
										Overlap = 1;
									}
								}
							}
						}

						for (int t = 0; t < NOBA; t++)
						{
							res = (static_cast<double>(BA[t].YA) - b) / a;

							if ((static_cast<double>(BA[t].XA) < res) && (res < static_cast<double>(BA[t].XB)))
							{
								if (node[i].Dy < vertex[s].Vy)
								{
									if (((node[i].Dy < static_cast<int>(BA[t].YA)) && (static_cast<int>(BA[t].YA) < vertex[s].Vy)))
									{
										Overlap = 1;
									}
								}
								else
								{
									if (((vertex[s].Vy < static_cast<int>(BA[t].YA)) && (static_cast<int>(BA[t].YA) < node[i].Dy)))
									{
										Overlap = 1;
									}
								}
							}
						}

						for (int t = 0; t < NOBA; t++)
						{
							res = (static_cast<double>(BA[t].YB) - b) / a;

							if ((static_cast<double>(BA[t].XA) < res) && (res < static_cast<double>(BA[t].XB)))
							{
								if (node[i].Dy < vertex[s].Vy)
								{
									if (((node[i].Dy < static_cast<int>(BA[t].YB)) && (static_cast<int>(BA[t].YB) < vertex[s].Vy)))
									{
										Overlap = 1;
									}
								}
								else
								{
									if (((vertex[s].Vy < static_cast<int>(BA[t].YB)) && (static_cast<int>(BA[t].YB) < node[i].Dy)))
									{
										Overlap = 1;
									}
								}
							}
						}
					}

					if (Overlap == 0)
					{
						Distance_x = static_cast<double>(fabs(node[i].Dx - vertex[s].Vx));
						Distance_y = static_cast<double>(fabs(node[i].Dy - vertex[s].Vy));

						Distance = hypot(Distance_x, Distance_y); //三平方の定理より2点間の距離を計算

						if (Min_AbsoluteValue > Distance)
						{
							result = s;
							Min_AbsoluteValue = Distance;
						}
					}
				}

				node[i].PD = result;

				Min_AbsoluteValue = UINT_MAX;

				if ((vertex[node[i].PD].Vx != vertex[node[i].PS].Vx) || (vertex[node[i].PD].Vy != vertex[node[i].PS].Vy))
				{
					break;
				}
			}

			node[i].RootNo = 0;

			SpeedVector = node[i].SP;
			X = abs(vertex[Root[i][node[i].RootNo]].Vx - node[i].Cx);
			Y = abs(vertex[Root[i][node[i].RootNo]].Vy - node[i].Cy);
			SquareRoot = pow(X, 2) + pow(Y, 2);
			node[i].Sx = SpeedVector * (X / sqrt(SquareRoot));
			node[i].Sy = SpeedVector * (Y / sqrt(SquareRoot));
			node[i].Sz = Z_SPEED;

			node[i].F = 1;

			DACost(i, NewGraph, vertex, Root, node, pCost);
		}
	}
}

unsigned int DACost(int NodeNumber, vector<Edge>* MST, Vertex* vertex, vector<int>* Root, Node* node, Cost* pCost)
{
	double X = 0;
	double Y = 0;
	unsigned int AverageCost = 0;

	pCost->C++;

	for (int i = 0; i < Root[NodeNumber].size() - 1; i++)
	{
		X = fabs(vertex[Root[NodeNumber][i + 1]].Vx - vertex[Root[NodeNumber][i]].Vx);
		Y = fabs(vertex[Root[NodeNumber][i + 1]].Vy - vertex[Root[NodeNumber][i]].Vy);
		pCost->Ave = pCost->Ave + static_cast<int>(hypot(X, Y));
	}

	X = fabs(node[NodeNumber].Cx - vertex[node[NodeNumber].PS].Vx);
	Y = fabs(node[NodeNumber].Cy - vertex[node[NodeNumber].PS].Vy);
	pCost->Ave = pCost->Ave + static_cast<int>(hypot(X, Y));

	X = fabs(node[NodeNumber].Dx - vertex[node[NodeNumber].PD].Vx);
	Y = fabs(node[NodeNumber].Dy - vertex[node[NodeNumber].PD].Vy);
	pCost->Ave = pCost->Ave + static_cast<int>(hypot(X, Y));

	pCost->Ave = pCost->Ave + (2 * HIGHT_OF_ROAD);

	if (pCost->C == 10)
	{
		pCost->Ave = static_cast<int>(pCost->Ave / pCost->C);

		pCost->C = 0;
	}

	return AverageCost;
}

void DA(int NodeNumber, Vertex* vertex, vector<Edge>* graph, Dijkstra* Dij, vector<int>* Root, int Start, int Goal)
{
	Edge edge;

	int tmp1 = 0;
	double tmp2 = SERVICE_AREA_X * SERVICE_AREA_Y;
	int Total = 0;

	for (int i = 0; i < NUMBER_OF_VERTEX; i++)
	{
		Dij[i].Check = 0;
		Dij[i].TotalCost = SERVICE_AREA_X * SERVICE_AREA_Y;
		Dij[i].VertexBeforeMoving = 0;
	}

	Dij[Start].Check = 1;
	Dij[Start].TotalCost = 0.0;
	Dij[Start].VertexBeforeMoving = Start;

	while (1)
	{
		for (int i = 0; i < NUMBER_OF_VERTEX; i++)
		{
			if (Dij[i].Check == 1)
			{
				for (int s = 0; s < graph[i].size(); s++)
				{
					edge = graph[i][s];
					tmp2 = Dij[i].TotalCost + edge.cost;
					if (tmp2 < Dij[edge.to].TotalCost)
					{
						Dij[edge.to].TotalCost = tmp2;
						Dij[edge.to].VertexBeforeMoving = i;
					}
				}
			}
		}

		tmp2 = SERVICE_AREA_X * SERVICE_AREA_Y;

		for (int i = 0; i < NUMBER_OF_VERTEX; i++)
		{
			if (Dij[i].Check == 0)
			{
				if (Dij[i].TotalCost < tmp2)
				{
					tmp1 = i;
					tmp2 = Dij[i].TotalCost;
				}
			}
		}

		Dij[tmp1].Check = 1;

		for (int i = 0; i < NUMBER_OF_VERTEX; i++)
		{
			if (Dij[i].Check == 1)
			{
				Total++;
			}
		}

		if (Total == NUMBER_OF_VERTEX)
		{
			break;
		}
		else
		{
			Total = 0;
		}
	}

	Root[NodeNumber].clear();
	Root[NodeNumber].push_back(Goal);
	Root[NodeNumber].push_back(Dij[Goal].VertexBeforeMoving);

	while (1)
	{
		if (Root[NodeNumber].back() != Start)
		{
			Root[NodeNumber].push_back(Dij[Root[NodeNumber].back()].VertexBeforeMoving);
		}
		else
		{
			break;
		}
	}
}

void Moving(Vertex* vertex, vector<int>* Root, Node* node)
{
	//ノードの移動
	for (int i = 0; i < NODE; i++)
	{
		if (node[i].ZF == 0)
		{
			node[i].Cz = node[i].Cz + node[i].Sz;
			if (node[i].Cz >= HIGHT_OF_ROAD)
			{
				node[i].Cz = HIGHT_OF_ROAD;
				node[i].ZF = 1;
			}
		}

		if (node[i].ZF == 1)
		{
			MoveTo(i, vertex, Root, node);
		}

		if (node[i].ZF == 2)
		{
			node[i].Cz = node[i].Cz - node[i].Sz;
			if (node[i].Cz <= 0)
			{ //目的地に到着

				node[i].Cz = 0;
				node[i].ZF = 0;
				node[i].F = 0;
				Root[i].clear();
				node[i].PS = node[i].PD;
			}
		}
    }

	for (int i = 0; i < NODE; i++)
	{
		if ((node[i].Cx == vertex[node[i].PS].Vx) && (node[i].Cy == vertex[node[i].PS].Vy) && (node[i].F == 1))
		{ //移動開始位置から最も近い頂点に移動完了した場合
			node[i].F = 2;
			if (node[i].PS == node[i].PD)
			{
				node[i].F = 3;

				double SpeedVector, X, Y, SquareRoot;

				SpeedVector = node[i].SP;
				X = abs(node[i].Dx - node[i].Cx);
				Y = abs(node[i].Dy - node[i].Cy);
				SquareRoot = pow(X, 2) + pow(Y, 2);
				node[i].Sx = SpeedVector * (X / sqrt(SquareRoot));
				node[i].Sy = SpeedVector * (Y / sqrt(SquareRoot));
			}
		}
		else if ((node[i].Cx == vertex[node[i].PD].Vx) && (node[i].Cy == vertex[node[i].PD].Vy) && (node[i].F == 2))
		{ //目的地から最も近い頂点に移動完了した場合
			node[i].F = 3;

			double SpeedVector, X, Y, SquareRoot;

			SpeedVector = node[i].SP;
			X = abs(node[i].Dx - node[i].Cx);
			Y = abs(node[i].Dy - node[i].Cy);
			SquareRoot = pow(X, 2) + pow(Y, 2);
			node[i].Sx = SpeedVector * (X / sqrt(SquareRoot));
			node[i].Sy = SpeedVector * (Y / sqrt(SquareRoot));
		}
		else if ((node[i].Cx == vertex[Root[i][node[i].RootNo]].Vx) && (node[i].Cy == vertex[Root[i][node[i].RootNo]].Vy) && (node[i].F == 2))
		{
			node[i].RootNo++;

			double SpeedVector, X, Y, SquareRoot;

			SpeedVector = node[i].SP;
			X = abs(vertex[Root[i][node[i].RootNo]].Vx - node[i].Cx);
			Y = abs(vertex[Root[i][node[i].RootNo]].Vy - node[i].Cy);
			SquareRoot = pow(X, 2) + pow(Y, 2);
			node[i].Sx = SpeedVector * (X / sqrt(SquareRoot));
			node[i].Sy = SpeedVector * (Y / sqrt(SquareRoot));
		}
		else if ((node[i].Cx == node[i].Dx) && (node[i].Cy == node[i].Dy) && (node[i].F == 3))
		{ //仮想道路上の目的地へ移動完了した場合
			node[i].ZF = 2;
			
		}
	}
}

void MoveTo(int NodeNumber, Vertex* vertex, vector<int>* Root, Node* node)
{
	int DDICx = 0; //Destinations divided into cases 場合分けした移動先のx座標
	int DDICy = 0; //Destinations divided into cases 場合分けした移動先のy座標


	if (node[NodeNumber].F == 1)
	{ //移動開始位置から最も近い頂点へ移動
		DDICx = vertex[node[NodeNumber].PS].Vx;
		DDICy = vertex[node[NodeNumber].PS].Vy;
	}
	else if (node[NodeNumber].F == 2)
	{ //頂点を移動
		DDICx = vertex[Root[NodeNumber][node[NodeNumber].RootNo]].Vx;
		DDICy = vertex[Root[NodeNumber][node[NodeNumber].RootNo]].Vy;
	}
	else if (node[NodeNumber].F == 3)
	{ //目的地へ移動
		DDICx = node[NodeNumber].Dx;
		DDICy = node[NodeNumber].Dy;
	}

	//各ノードを移動させる
	if (node[NodeNumber].Cx < DDICx && node[NodeNumber].Cy < DDICy) //1
	{
		node[NodeNumber].Cx = node[NodeNumber].Cx + node[NodeNumber].Sx;
		node[NodeNumber].Cy = node[NodeNumber].Cy + node[NodeNumber].Sy;

		if (DDICx < node[NodeNumber].Cx && node[NodeNumber].Cy < DDICy)
		{
			node[NodeNumber].Cx = DDICx;
		}
		else if (node[NodeNumber].Cx < DDICx && DDICy < node[NodeNumber].Cy)
		{
			node[NodeNumber].Cy = DDICy;
		}
		else if (DDICx < node[NodeNumber].Cx && DDICy < node[NodeNumber].Cy)
		{
			node[NodeNumber].Cx = DDICx;
			node[NodeNumber].Cy = DDICy;
		}
	}
	else if (node[NodeNumber].Cx < DDICx && node[NodeNumber].Cy == DDICy) //2
	{
		node[NodeNumber].Cx = node[NodeNumber].Cx + node[NodeNumber].Sx;

		if (DDICx < node[NodeNumber].Cx)
		{
			node[NodeNumber].Cx = DDICx;
		}
	}
	else if (node[NodeNumber].Cx < DDICx && DDICy < node[NodeNumber].Cy) //3
	{
		node[NodeNumber].Cx = node[NodeNumber].Cx + node[NodeNumber].Sx;
		node[NodeNumber].Cy = node[NodeNumber].Cy - node[NodeNumber].Sy;

		if (node[NodeNumber].Cx < DDICx && node[NodeNumber].Cy < DDICy)
		{
			node[NodeNumber].Cy = DDICy;
		}
		else if (DDICx < node[NodeNumber].Cx && DDICy < node[NodeNumber].Cy)
		{
			node[NodeNumber].Cx = DDICx;
		}
		else if (DDICx < node[NodeNumber].Cx && node[NodeNumber].Cy < DDICy)
		{
			node[NodeNumber].Cx = DDICx;
			node[NodeNumber].Cy = DDICy;
		}
	}
	else if (node[NodeNumber].Cx == DDICx && node[NodeNumber].Cy < DDICy) //4
	{
		node[NodeNumber].Cy = node[NodeNumber].Cy + node[NodeNumber].Sy;

		if (DDICy < node[NodeNumber].Cy)
		{
			node[NodeNumber].Cy = DDICy;
		}
	}
	else if (node[NodeNumber].Cx == DDICx && DDICy < node[NodeNumber].Cy) //5
	{
		node[NodeNumber].Cy = node[NodeNumber].Cy - node[NodeNumber].Sy;

		if (node[NodeNumber].Cy < DDICy)
		{
			node[NodeNumber].Cy = DDICy;
		}
	}
	else if (DDICx < node[NodeNumber].Cx && node[NodeNumber].Cy < DDICy) //6
	{
		node[NodeNumber].Cx = node[NodeNumber].Cx - node[NodeNumber].Sx;
		node[NodeNumber].Cy = node[NodeNumber].Cy + node[NodeNumber].Sy;

		if (node[NodeNumber].Cx < DDICx && node[NodeNumber].Cy < DDICy)
		{
			node[NodeNumber].Cx = DDICx;
		}
		else if (DDICx < node[NodeNumber].Cx && DDICy < node[NodeNumber].Cy)
		{
			node[NodeNumber].Cy = DDICy;
		}
		else if (node[NodeNumber].Cx < DDICx && DDICy < node[NodeNumber].Cy)
		{
			node[NodeNumber].Cx = DDICx;
			node[NodeNumber].Cy = DDICy;
		}
	}
	else if (DDICx < node[NodeNumber].Cx && node[NodeNumber].Cy == DDICy) //7
	{
		node[NodeNumber].Cx = node[NodeNumber].Cx - node[NodeNumber].Sx;

		if (node[NodeNumber].Cx < DDICx)
		{
			node[NodeNumber].Cx = DDICx;
		}
	}
	else //8
	{
		node[NodeNumber].Cx = node[NodeNumber].Cx - node[NodeNumber].Sx;
		node[NodeNumber].Cy = node[NodeNumber].Cy - node[NodeNumber].Sy;

		if (DDICx < node[NodeNumber].Cx && node[NodeNumber].Cy < DDICy)
		{
			node[NodeNumber].Cy = DDICy;
		}
		else if (node[NodeNumber].Cx < DDICx && DDICy < node[NodeNumber].Cy)
		{
			node[NodeNumber].Cx = DDICx;
		}
		else if (node[NodeNumber].Cx < DDICx && node[NodeNumber].Cy < DDICy)
		{
			node[NodeNumber].Cx = DDICx;
			node[NodeNumber].Cy = DDICy;
		}
	}
}