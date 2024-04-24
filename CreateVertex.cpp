#include "CreateVertex.h"

bool VeCos(const struct Edge& alpha, const struct Edge& beta) {
	return alpha.cost < beta.cost;
}

void CreateVertex(Vertex* vertex, vector<Edge>* graph, BanedAreas* BA, int TryTimes)
{
	CreateBanedAreas(BA); //飛行禁止エリアの生成

	CreateVertex(vertex, BA, TryTimes); //頂点の生成

	SortVertex(vertex); //頂点を反時計回りにソート

	CreateEdge(vertex, BA, graph); //辺の生成
}

void CreateBanedAreas(BanedAreas* BA)
{

	BA[0].XA = 250;
	BA[0].XB = 500;
	BA[0].YA = 250;
	BA[0].YB = 500;

	BA[1].XA = 500;
	BA[1].XB = 750;
	BA[1].YA = 500;
	BA[1].YB = 750;

	/*BA[1].XA = 6000;
	BA[1].XB = 10000;
	BA[1].YA = 4500;
	BA[1].YB = 5500;

	BA[2].XA = 4500;
	BA[2].XB = 5500;
	BA[2].YA = 0;
	BA[2].YB = 4000;

	BA[3].XA = 4500;
	BA[3].XB = 5500;
	BA[3].YA = 6000;
	BA[3].YB = 10000;*/
}

void CreateVertex(Vertex* vertex, BanedAreas* BA, int TryTimes)
{
	int Overlap = 0;
	double Distance_x = 0.0; //頂点同士のx方向の距離 
	double Distance_y = 0.0; //頂点同士のy方向の距離
	int TT = TryTimes;

	mt19937 mt(TT); //擬似乱数の初期シードとして生成したオブジェクトを設定
	uniform_int_distribution<int> GenerateVertexX(0, SERVICE_AREA_X); //生成する頂点のx座標の範囲を設定
	uniform_int_distribution<int> GenerateVertexY(0, SERVICE_AREA_Y); //生成する頂点のy座標の範囲を設定


			for (int i = 0; i < NUMBER_OF_VERTEX; i++)
			{
				do
				{
					Overlap = 0;

					vertex[i].Vx = GenerateVertexX(mt);
					vertex[i].Vy = GenerateVertexY(mt);

					for (int s = 0; s < NOBA; s++)
					{
						if (((static_cast<int>(BA[s].XA) < vertex[i].Vx) && (vertex[i].Vx < static_cast<int>(BA[s].XB))) && ((static_cast<int>(BA[s].YA) < vertex[i].Vy) && (vertex[i].Vy < static_cast<int>(BA[s].YB))))
						{
							Overlap = 1;
						}
					}


					for (int s = 0; s < i; s++)
					{
						if ((vertex[s].Vx == vertex[i].Vx) && (vertex[s].Vy == vertex[i].Vy))
						{
							Overlap = 1;

							continue;
						}
					}

				} while (Overlap);

				Distance_x = static_cast<double>(fabs(vertex[i].Vx - static_cast<int>(SERVICE_AREA_X / 2)));
				Distance_y = static_cast<double>(fabs(vertex[i].Vy - static_cast<int>(SERVICE_AREA_Y / 2)));

				vertex[i].DisCen = hypot(Distance_x, Distance_y);
			}
	
}

void SortVertex(Vertex* vertex)
{
	int tmp1 = 0; double tmp2 = 0.0;
	double Distance_x = 0.0; //頂点同士のx方向の距離 
	double Distance_y = 0.0; //頂点同士のy方向の距離

	//中心に近い順にソート
	for (int i = 0; i < NUMBER_OF_VERTEX; i++) {
		for (int s = i + 1; s < NUMBER_OF_VERTEX; s++) {
			if (vertex[i].DisCen > vertex[s].DisCen) {
				tmp1 = vertex[i].Vx;
				vertex[i].Vx = vertex[s].Vx;
				vertex[s].Vx = tmp1;

				tmp1 = vertex[i].Vy;
				vertex[i].Vy = vertex[s].Vy;
				vertex[s].Vy = tmp1;

				tmp2 = vertex[i].DisCen;
				vertex[i].DisCen = vertex[s].DisCen;
				vertex[s].DisCen = tmp2;
			}
		}
	}

	//反時計回りにソート
	for (int i = 1; i < NUMBER_OF_VERTEX; i++) {
		Distance_x = static_cast<double>(vertex[i].Vx - vertex[0].Vx);
		Distance_y = static_cast<double>(vertex[i].Vy - vertex[0].Vy);

		vertex[i].DisCen = atan2(Distance_y, Distance_x);
	}

	for (int i = 1; i < NUMBER_OF_VERTEX; i++) {
		for (int s = i + 1; s < NUMBER_OF_VERTEX; s++) {
			if (vertex[i].DisCen > vertex[s].DisCen) {
				tmp1 = vertex[i].Vx;
				vertex[i].Vx = vertex[s].Vx;
				vertex[s].Vx = tmp1;

				tmp1 = vertex[i].Vy;
				vertex[i].Vy = vertex[s].Vy;
				vertex[s].Vy = tmp1;

				tmp2 = vertex[i].DisCen;
				vertex[i].DisCen = vertex[s].DisCen;
				vertex[s].DisCen = tmp2;
			}
		}
	}
}

void CreateEdge(Vertex* vertex, BanedAreas* BA, vector<Edge>* graph)
{
	Edge edge;
	int Overlap = 0;
	double Distance_x = 0.0; //頂点同士のx方向の距離 
	double Distance_y = 0.0; //頂点同士のy方向の距離
	double Distance; //頂点同士のユークリッド距離
	double a = 0.0, b = 0.0, res = 0.0;

	for (int i = 0; i < NUMBER_OF_VERTEX; i++)
	{
		for (int s = 0; s < NUMBER_OF_VERTEX; s++)
		{
			if (i != s)
			{
				Overlap = 0;

				Distance_x = fabs(vertex[i].Vx - vertex[s].Vx);
				Distance_y = fabs(vertex[i].Vy - vertex[s].Vy);

				Distance = hypot(Distance_x, Distance_y); //三平方の定理より2点間の距離を計算

				if (vertex[i].Vx == vertex[s].Vx)
				{
					if (vertex[i].Vy < vertex[s].Vy)
					{
						for (int t = 0; t < NOBA; t++)
						{
							if (!((vertex[i].Vy < BA[t].YA) && (BA[t].YB < vertex[s].Vy)))
							{
								Overlap = 1;
							}
						}
					}
					else
					{
						for (int t = 0; t < NOBA; t++)
						{
							if (!((vertex[s].Vy < BA[t].YA) && (BA[t].YB < vertex[i].Vy)))
							{
								Overlap = 1;
							}
						}
					}
				}
				else if (vertex[i].Vy == vertex[s].Vy)
				{
					if (vertex[i].Vx < vertex[s].Vx)
					{
						for (int t = 0; t < NOBA; t++)
						{
							if (!((vertex[i].Vx < BA[t].XA) && (BA[t].XB < vertex[s].Vx)))
							{
								Overlap = 1;
							}
						}
					}
					else
					{
						for (int t = 0; t < NOBA; t++)
						{
							if (!((vertex[s].Vx < BA[t].XA) && (BA[t].XB < vertex[i].Vx)))
							{
								Overlap = 1;
							}
						}
					}
				}
				else
				{
					a = static_cast<double>(vertex[s].Vy - vertex[i].Vy) / static_cast<double>(vertex[s].Vx - vertex[i].Vx);
					b = static_cast<double>(vertex[i].Vy) - a * static_cast<double>(vertex[i].Vx);

					for (int t = 0; t < NOBA; t++)
					{
						res = a * static_cast<double>(BA[t].XA) + b;

						if ((static_cast<double>(BA[t].YA) < res) && (res < static_cast<double>(BA[t].YB)))
						{
							if (vertex[i].Vx < vertex[s].Vx)
							{
								if (((vertex[i].Vx < static_cast<int>(BA[t].XA)) && (static_cast<int>(BA[t].XA) < vertex[s].Vx)))
								{
									Overlap = 1;
								}

							}
							else
							{
								if (((vertex[s].Vx < static_cast<int>(BA[t].XA)) && (static_cast<int>(BA[t].XA) < vertex[i].Vx)))
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
							if (vertex[i].Vx < vertex[s].Vx)
							{
								if (((vertex[i].Vx < static_cast<int>(BA[t].XB)) && (static_cast<int>(BA[t].XB) < vertex[s].Vx)))
								{
									Overlap = 1;
								}
							}
							else
							{
								if (((vertex[s].Vx < static_cast<int>(BA[t].XB)) && (static_cast<int>(BA[t].XB) < vertex[i].Vx)))
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
							if (vertex[i].Vy < vertex[s].Vy)
							{
								if (((vertex[i].Vy < static_cast<int>(BA[t].YA)) && (static_cast<int>(BA[t].YA) < vertex[s].Vy)))
								{
									Overlap = 1;
								}
							}
							else
							{
								if (((vertex[s].Vy < static_cast<int>(BA[t].YA)) && (static_cast<int>(BA[t].YA) < vertex[i].Vy)))
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
							if (vertex[i].Vy < vertex[s].Vy)
							{
								if (((vertex[i].Vy < static_cast<int>(BA[t].YB)) && (static_cast<int>(BA[t].YB) < vertex[s].Vy)))
								{
									Overlap = 1;
								}
							}
							else
							{
								if (((vertex[s].Vy < static_cast<int>(BA[t].YB)) && (static_cast<int>(BA[t].YB) < vertex[i].Vy)))
								{
									Overlap = 1;
								}
							}
						}
					}
				}

				if (Overlap == 0)
				{
					edge = { s,Distance };

					graph[i].push_back(edge);
				}
			}
		}

		sort(graph[i].begin(), graph[i].end(), VeCos);
	}
}

void DijkstraMethod(Vertex* vertex, vector<Edge>* graph, Dijkstra* Dij, vector<Edge>* NewGraph)
{
	Edge edge;

	Dij[0].VertexBeforeMoving = 0;
	Dij[0].TotalCost = 0;
	Dij[0].Check = 1;

	int tmp1 = 0, tmp3 = 0;
	double tmp2 = 0.0;

	for (int i = 1; i < NUMBER_OF_VERTEX; i++)
	{
		Dij[i].VertexBeforeMoving = -1;
		Dij[i].TotalCost = SERVICE_AREA_X * SERVICE_AREA_Y;
		Dij[i].Check = 0;
	}

	edge = graph[0][0];

	Dij[edge.to].VertexBeforeMoving = 0;
	Dij[edge.to].TotalCost = edge.cost;
	Dij[edge.to].Check = 1;

	for (int sum = 0; sum < NUMBER_OF_VERTEX - 2; sum++)
	{
		tmp2 = SERVICE_AREA_X * SERVICE_AREA_Y;

		for (int i = 0; i < NUMBER_OF_VERTEX; i++)
		{
			if (Dij[i].Check == 0)
			{
				for (int s = 0; s < NUMBER_OF_VERTEX; s++)
				{
					if (Dij[s].Check == 1)
					{
						for (int t = 0; t < graph[s].size(); t++)
						{
							edge = graph[s][t];

							if (i == edge.to)
							{
								if (tmp2 > edge.cost + Dij[s].TotalCost)
								{
									tmp1 = s;
									tmp2 = edge.cost + Dij[s].TotalCost;
									tmp3 = i;
								}
							}
						}
					}
				}
			}
		}

		Dij[tmp3].VertexBeforeMoving = tmp1;
		Dij[tmp3].TotalCost = tmp2;
		Dij[tmp3].Check = 1;
	}

	for (int i = 0; i < NUMBER_OF_VERTEX; i++)
	{
		for (int s = 0; s < NUMBER_OF_VERTEX; s++)
		{
			if (i != s)
			{
				if (i == Dij[s].VertexBeforeMoving)
				{
					tmp2 = hypot(fabs(vertex[i].Vx - vertex[s].Vx), fabs(vertex[i].Vy - vertex[s].Vy)); //三平方の定理より2点間の距離を計算

					edge = { s,tmp2 };

					NewGraph[i].push_back(edge);

					edge = { i,tmp2 };

					NewGraph[s].push_back(edge);
				}
			}
		}
	}
}

int DFS(int NodeNumber, vector<Edge>* NewGraph, vector<int>* Root, int Start, int Destination)
{
	Edge edge;
	vector<bool> Seen = { 0 };
	vector<int> ToDo;
	vector<bool> SeenToDo = { 0 };
	bool Leaf;
	bool Check;
	int CurrentEdge;
	int DFSOut = 0;

	Seen.assign(NUMBER_OF_VERTEX, 0); //Seen全体をFALSEに初期化
	SeenToDo.assign(NUMBER_OF_VERTEX, 0);
	ToDo.erase(ToDo.begin(), ToDo.end()); //ToDo全体を空に
	Seen[Start] = 1; //探索する根をTRUEに変換
	ToDo.push_back(Start); //ToDoに根を追加

	while (1)
	{
		if (ToDo.empty())
		{
			DFSOut = 1;

			break;
		}

		Leaf = 1;
		Check = 1;

		CurrentEdge = ToDo.back(); //ToDoから頂点を1つ取り出し，探索中の頂点とする
		ToDo.erase(ToDo.end() - 1);

		for (int i = 0; i < NewGraph[CurrentEdge].size(); i++)
		{
			edge = NewGraph[CurrentEdge][i];

			if (Seen[edge.to] != 1)
			{
				Seen[edge.to] = 1;
				ToDo.push_back(edge.to);
				Leaf = 0;
			}
		}

		Root[NodeNumber].push_back(CurrentEdge);
		SeenToDo[CurrentEdge] = 1;

		if (CurrentEdge == Destination)
		{
			break;
		}

		if (Leaf == 1)
		{
			Root[NodeNumber].erase(Root[NodeNumber].end() - 1);

			while (1)
			{
				if (Root[NodeNumber].empty())
				{
					DFSOut = 1;

					break;
				}

				for (int i = 0; i < NewGraph[Root[NodeNumber].back()].size(); i++)
				{
					for (int s = 0; s < NUMBER_OF_VERTEX; s++)
					{
						edge = NewGraph[Root[NodeNumber].back()][i];

						if ((edge.to == s) && (SeenToDo[s] == 0))
						{
							Check = 0;

							break;
						}
					}

					if (Check == 0)
					{
						break;
					}
				}

				if (Check == 0)
				{
					break;
				}

				Root[NodeNumber].erase(Root[NodeNumber].end() - 1);
			}
		}
	}

	return DFSOut;
}

void CreateCircleEdge(Vertex* vertex, BanedAreas* BA, vector<Edge>* NewGraph)
{
	Edge edge;
	int Overlap = 0, s = 0;
	double Distance_x = 0.0; //頂点同士のx方向の距離 
	double Distance_y = 0.0; //頂点同士のy方向の距離
	double Distance; //頂点同士のユークリッド距離
	double a = 0.0, b = 0.0, res = 0.0;

	for (int i = 1; i < NUMBER_OF_VERTEX; i++)
	{
		Overlap = 0;

		if (i == (NUMBER_OF_VERTEX - 1))
		{
			s = 1;
			Distance_x = fabs(vertex[i].Vx - vertex[1].Vx);
			Distance_y = fabs(vertex[i].Vy - vertex[1].Vy);
		}
		else
		{
			s = i + 1;
			Distance_x = fabs(vertex[i].Vx - vertex[i + 1].Vx);
			Distance_y = fabs(vertex[i].Vy - vertex[i + 1].Vy);
		}

		Distance = hypot(Distance_x, Distance_y); //三平方の定理より2点間の距離を計算

		if (vertex[i].Vx == vertex[s].Vx)
		{
			if (vertex[i].Vy < vertex[s].Vy)
			{
				for (int t = 0; t < NOBA; t++)
				{
					if (!((vertex[i].Vy < BA[t].YA) && (BA[t].YB < vertex[s].Vy)))
					{
						Overlap = 1;
					}
				}
			}
			else
			{
				for (int t = 0; t < NOBA; t++)
				{
					if (!((vertex[s].Vy < BA[t].YA) && (BA[t].YB < vertex[i].Vy)))
					{
						Overlap = 1;
					}
				}
			}
		}
		else if (vertex[i].Vy == vertex[s].Vy)
		{
			if (vertex[i].Vx < vertex[s].Vx)
			{
				for (int t = 0; t < NOBA; t++)
				{
					if (!((vertex[i].Vx < BA[t].XA) && (BA[t].XB < vertex[s].Vx)))
					{
						Overlap = 1;
					}
				}
			}
			else
			{
				for (int t = 0; t < NOBA; t++)
				{
					if (!((vertex[s].Vx < BA[t].XA) && (BA[t].XB < vertex[i].Vx)))
					{
						Overlap = 1;
					}
				}
			}
		}
		else
		{
			a = static_cast<double>(vertex[s].Vy - vertex[i].Vy) / static_cast<double>(vertex[s].Vx - vertex[i].Vx);
			b = static_cast<double>(vertex[i].Vy) - a * static_cast<double>(vertex[i].Vx);

			for (int t = 0; t < NOBA; t++)
			{
				res = a * static_cast<double>(BA[t].XA) + b;

				if ((static_cast<double>(BA[t].YA) < res) && (res < static_cast<double>(BA[t].YB)))
				{
					if (vertex[i].Vx < vertex[s].Vx)
					{
						if (((vertex[i].Vx < static_cast<int>(BA[t].XA)) && (static_cast<int>(BA[t].XA) < vertex[s].Vx)))
						{
							Overlap = 1;
						}

					}
					else
					{
						if (((vertex[s].Vx < static_cast<int>(BA[t].XA)) && (static_cast<int>(BA[t].XA) < vertex[i].Vx)))
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
					if (vertex[i].Vx < vertex[s].Vx)
					{
						if (((vertex[i].Vx < static_cast<int>(BA[t].XB)) && (static_cast<int>(BA[t].XB) < vertex[s].Vx)))
						{
							Overlap = 1;
						}
					}
					else
					{
						if (((vertex[s].Vx < static_cast<int>(BA[t].XB)) && (static_cast<int>(BA[t].XB) < vertex[i].Vx)))
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
					if (vertex[i].Vy < vertex[s].Vy)
					{
						if (((vertex[i].Vy < static_cast<int>(BA[t].YA)) && (static_cast<int>(BA[t].YA) < vertex[s].Vy)))
						{
							Overlap = 1;
						}
					}
					else
					{
						if (((vertex[s].Vy < static_cast<int>(BA[t].YA)) && (static_cast<int>(BA[t].YA) < vertex[i].Vy)))
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
					if (vertex[i].Vy < vertex[s].Vy)
					{
						if (((vertex[i].Vy < static_cast<int>(BA[t].YB)) && (static_cast<int>(BA[t].YB) < vertex[s].Vy)))
						{
							Overlap = 1;
						}
					}
					else
					{
						if (((vertex[s].Vy < static_cast<int>(BA[t].YB)) && (static_cast<int>(BA[t].YB) < vertex[i].Vy)))
						{
							Overlap = 1;
						}
					}
				}

				if (Overlap == 0)
				{
					edge = { s,Distance };
					NewGraph[i].push_back(edge);
					edge = { i,Distance };
					NewGraph[s].push_back(edge);
				}
			}
		}
	}
}