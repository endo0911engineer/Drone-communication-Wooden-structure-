#include "Database.h"
#include "CreateVertex.h"
#include "NodeInfo.h"
#include "Connection.h"

void FileOperationStart(ofstream& NOC);
void FileOperationFinish(ofstream& NOC, Cost* pCost, int** NumberOfConnections, ofstream& CA, RoCo* pRoco);
void GraphCheck(Vertex* vertex, vector<Edge>* graph, Dijkstra* Dij, vector<Edge>* NewGraph, vector<int>* Root, BanedAreas* BA, int TryTimes);
void Clock(int* pTime);

// プログラムは WinMain から開始
int main()
{
	ofstream NOC("NumberOfContactAndMovingCost.txt", ios::app); //総コンタクト回数
	ofstream CA("ContactArea.txt", ios::app); //コンタクト場所情報
	FileOperationStart(NOC); //ファイルの初期設定

	for (int Count = 0; Count < 10; Count++)
	{
		cout << Count << endl;

		int Time[4] = { 0 }; //時間の情報を格納
		Vertex vertex[NUMBER_OF_VERTEX]; //各頂点の座標を格納
		vector<Edge> graph[NUMBER_OF_VERTEX]; //各頂点間のcost情報
		Dijkstra Dij[NUMBER_OF_VERTEX]; //ダイクストラ法の結果
		vector<Edge> NewGraph[NUMBER_OF_VERTEX]; //条件処理済みのグラフ情報
		vector<int> Root[NODE + 1]; //各ノードの経路
		Node node[NODE];
		Cost cost;
		Cost* pCost = &cost;
		RoCo roco;
		RoCo* pRoco = &roco;
		BanedAreas BA[NOBA]; //飛行禁止エリア
		int TryTimes = 1;

		double* Distance[NODE - 1];
		double* Distance_x[NODE - 1];
		double* Distance_y[NODE - 1];
		double* Distance_z[NODE - 1];
		int* ConnectedPointFlag[NODE - 1];
		int* NumberOfConnections[NODE - 1];

		for (int i = 0; i < NODE - 1; i++)
		{
			Distance[i] = new double[NODE - 1 - i];
			Distance_x[i] = new double[NODE - 1 - i];
			Distance_y[i] = new double[NODE - 1 - i];
			Distance_z[i] = new double[NODE - 1 - i];
			ConnectedPointFlag[i] = new int[NODE - 1 - i];
			NumberOfConnections[i] = new int[NODE - 1 - i];

			for (int s = 0; s < NODE - i - 1; s++)
			{
				ConnectedPointFlag[i][s] = 0;
				NumberOfConnections[i][s] = 0;
			}
		}

		GraphCheck(vertex, graph, Dij, NewGraph, Root, BA, TryTimes); //グラフ生成
		//CreateCircleEdge(vertex, BA, NewGraph); //グラフに外周円の道を生成
		GenerateNode(vertex, NewGraph, node, BA);

		while (1)
		{
			Time[0]++; //時間を進める
			Clock(Time); //経過時間の表示
			DestinationAndSpeedGeneration(vertex, NewGraph, node, pCost, Root, BA, Dij);
			Moving(vertex, Root, node);
			CalculateDistanceBetweenTwoPoints(node, Distance, Distance_x, Distance_y, Distance_z);
			ConnectTwoPoints(node, Distance, ConnectedPointFlag, NumberOfConnections, pRoco);

			//キーボードZを押すことでシミュレーションを終了する
			if (Time[3] == 100)
			{
				break;
			}
		}

		FileOperationFinish(NOC, pCost, NumberOfConnections, CA, pRoco); //ファイルの終了処理
	}

	return 0;				// ソフトの終了 
}

void FileOperationStart(ofstream& NOC)
{
	NOC << "NODE" << "," << NODE << endl;
	NOC << "RANGE" << "," << RANGE << endl;
	NOC << "VERTEX" << "," << NUMBER_OF_VERTEX << endl;
}

void FileOperationFinish(ofstream& NOC, Cost* pCost, int** NumberOfConnections, ofstream& CA, RoCo* pRoco)
{
	int SumConnections = 0; //総コンタクト回数

	if (pCost->C != 0)
	{
		pCost->Ave = static_cast<int>(pCost->Ave / pCost->C);
	}

	for (int i = 0; i < NODE - 1; i++)
	{
		for (int s = 0; s < NODE - i - 1; s++)
		{
			SumConnections = SumConnections + NumberOfConnections[i][s];
		}
	}

	NOC << SumConnections << "," << pCost->Ave << endl;
	CA << pRoco->OnRoad << "," << pRoco->NoOnRoad << endl;
}

void GraphCheck(Vertex* vertex, vector<Edge>* graph, Dijkstra* Dij, vector<Edge>* NewGraph, vector<int>* Root, BanedAreas* BA, int TryTimes)
{
	int Check = 1;

	do
	{ 
     	TryTimes++;

		for (int i = 0; i < NUMBER_OF_VERTEX; i++)
		{
			graph[i].clear();
			NewGraph[i].clear();
		}

		CreateVertex(vertex, graph, BA, TryTimes);
		DijkstraMethod(vertex, graph, Dij, NewGraph);

		Check = 1;

		for (int i = 0; i < NUMBER_OF_VERTEX; i++)
		{
			Check = DFS(0, NewGraph, Root, 0, i);
			Root[0].clear();

			if (Check == 1)
			{
				break;
			}
		}

	} while (Check);
}

void Clock(int* pTime)
{
	if (pTime[0] == 10) {
		//10[ms]経過したら1[s]増やす
		pTime[1]++;

		pTime[0] = 0;

		if (pTime[1] == 60) {
			//60[s]経過したら1[m]増やす
			pTime[2]++;

			pTime[1] = 0;

			if (pTime[2] == 60) {
				//60[m]経過したら1[h]増やす
				pTime[3]++;

				pTime[2] = 0;
			}
		}
	}
}