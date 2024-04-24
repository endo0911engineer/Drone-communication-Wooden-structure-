#include "Database.h"
#include "CreateVertex.h"
#include "NodeInfo.h"
#include "Connection.h"

void FileOperationStart(ofstream& NOC);
void FileOperationFinish(ofstream& NOC, Cost* pCost, int** NumberOfConnections, ofstream& CA, RoCo* pRoco);
void GraphCheck(Vertex* vertex, vector<Edge>* graph, Dijkstra* Dij, vector<Edge>* NewGraph, vector<int>* Root, BanedAreas* BA, int TryTimes);
void Clock(int* pTime);

// �v���O������ WinMain ����J�n
int main()
{
	ofstream NOC("NumberOfContactAndMovingCost.txt", ios::app); //���R���^�N�g��
	ofstream CA("ContactArea.txt", ios::app); //�R���^�N�g�ꏊ���
	FileOperationStart(NOC); //�t�@�C���̏����ݒ�

	for (int Count = 0; Count < 10; Count++)
	{
		cout << Count << endl;

		int Time[4] = { 0 }; //���Ԃ̏����i�[
		Vertex vertex[NUMBER_OF_VERTEX]; //�e���_�̍��W���i�[
		vector<Edge> graph[NUMBER_OF_VERTEX]; //�e���_�Ԃ�cost���
		Dijkstra Dij[NUMBER_OF_VERTEX]; //�_�C�N�X�g���@�̌���
		vector<Edge> NewGraph[NUMBER_OF_VERTEX]; //���������ς݂̃O���t���
		vector<int> Root[NODE + 1]; //�e�m�[�h�̌o�H
		Node node[NODE];
		Cost cost;
		Cost* pCost = &cost;
		RoCo roco;
		RoCo* pRoco = &roco;
		BanedAreas BA[NOBA]; //��s�֎~�G���A
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

		GraphCheck(vertex, graph, Dij, NewGraph, Root, BA, TryTimes); //�O���t����
		//CreateCircleEdge(vertex, BA, NewGraph); //�O���t�ɊO���~�̓��𐶐�
		GenerateNode(vertex, NewGraph, node, BA);

		while (1)
		{
			Time[0]++; //���Ԃ�i�߂�
			Clock(Time); //�o�ߎ��Ԃ̕\��
			DestinationAndSpeedGeneration(vertex, NewGraph, node, pCost, Root, BA, Dij);
			Moving(vertex, Root, node);
			CalculateDistanceBetweenTwoPoints(node, Distance, Distance_x, Distance_y, Distance_z);
			ConnectTwoPoints(node, Distance, ConnectedPointFlag, NumberOfConnections, pRoco);

			//�L�[�{�[�hZ���������ƂŃV�~�����[�V�������I������
			if (Time[3] == 100)
			{
				break;
			}
		}

		FileOperationFinish(NOC, pCost, NumberOfConnections, CA, pRoco); //�t�@�C���̏I������
	}

	return 0;				// �\�t�g�̏I�� 
}

void FileOperationStart(ofstream& NOC)
{
	NOC << "NODE" << "," << NODE << endl;
	NOC << "RANGE" << "," << RANGE << endl;
	NOC << "VERTEX" << "," << NUMBER_OF_VERTEX << endl;
}

void FileOperationFinish(ofstream& NOC, Cost* pCost, int** NumberOfConnections, ofstream& CA, RoCo* pRoco)
{
	int SumConnections = 0; //���R���^�N�g��

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
		//10[ms]�o�߂�����1[s]���₷
		pTime[1]++;

		pTime[0] = 0;

		if (pTime[1] == 60) {
			//60[s]�o�߂�����1[m]���₷
			pTime[2]++;

			pTime[1] = 0;

			if (pTime[2] == 60) {
				//60[m]�o�߂�����1[h]���₷
				pTime[3]++;

				pTime[2] = 0;
			}
		}
	}
}