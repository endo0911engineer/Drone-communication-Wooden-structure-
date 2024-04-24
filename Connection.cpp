#include "Connection.h"

void CalculateDistanceBetweenTwoPoints(Node* node, double** Distance, double** Distance_x, double** Distance_y, double** Distance_z)
{
	for (int i = 0; i < NODE - 1; i++)
	{
		for (int s = 0; s < NODE - i - 1; s++)
		{
			Distance_x[i][s] = fabs(node[i].Cx - node[s + i + 1].Cx);
			Distance_y[i][s] = fabs(node[i].Cy - node[s + i + 1].Cy);
			Distance_z[i][s] = fabs(node[i].Cz - node[s + i + 1].Cz);

            Distance[i][s] = sqrt(pow(Distance_x[i][s], 2) + pow(Distance_y[i][s], 2) + pow(Distance_z[i][s], 2)); //�O�����̒藝���2�_�Ԃ̋������v�Z
		}
	}
}

void ConnectTwoPoints(Node* node, double** Distance, int** ConnectedPointFlag, int** NumberOfConnections, RoCo* pRoco)
{
	for (int i = 0; i < NODE - 1; i++)
	{
		for (int s = 0; s < NODE - i - 1; s++)
		{
			if (Distance[i][s] < RANGE)
			{
				//�q�����Ă��Ȃ���Ԃ���q�������ꍇ
				if (ConnectedPointFlag[i][s] == 0)
				{
					ConnectedPointFlag[i][s] = 1;

					NumberOfConnections[i][s]++;

					//��������R���^�N�g�ꏊ�̏��𓾂�

					if ((node[i].F == 2) || (node[i].F == 3))
					{
						pRoco->OnRoad++;
					}
					else
					{
						pRoco->NoOnRoad++;
					}

					if ((node[s + i + 1].F == 2) || (node[s + i + 1].F == 3))
					{
						pRoco->OnRoad++;
					}
					else
					{
						pRoco->NoOnRoad++;
					}
				}
			}
			else
			{
				//�q�����Ă����Ԃ���r�؂ꂽ�ꍇ
				if (ConnectedPointFlag[i][s] == 1)
				{
					ConnectedPointFlag[i][s] = 0;
				}
			}
		}
	}
}