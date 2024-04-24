#pragma once
#include <vector>
#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <fstream>
#include <iostream>

using namespace std;

#define NODE 2 //�m�[�h�̐�
#define MIN_SPEED 30 //�m�[�h�̍ō����x
#define MAX_SPEED 30 //�m�[�h�̍Œᑬ�x
#define Z_SPEED 10 //z���̑��x
#define RANGE 100 //�ʐM�\����
#define NUMBER_OF_VERTEX 25 //���_�̐�
#define SERVICE_AREA_X 1000 //�T�[�r�X�G���A��x�����͈̔�
#define SERVICE_AREA_Y 1000 //�T�[�r�X�G���A��y�����͈̔�
#define HIGHT_OF_ROAD 1000 //���̉��z���H�̍���
#define NOBA 2 //NUMBER_OF_BANED_AREAS(��s�֎~�G���A�̐�)

struct Vertex {
	int Vx; //���_��x���W
	int Vy; //���_��y���W
	double DisCen; //��������̊e���_�ւ̋����ƃV�[�^�̒l
};

struct Edge {
	int to; //�אڃm�[�h
	double cost; //�אڃm�[�h�ւ�cost
};

struct Dijkstra {
	int VertexBeforeMoving;
	double TotalCost;
	int Check;
};

struct Node {
	double Cx; //�m�[�h��x���W
	double Cy; //�m�[�h��y���W
	double Cz; //�m�[�h��z���W
	int Dx; //�m�[�h�̖ړI�n��x���W
	int Dy; //�m�[�h�̖ړI�n��y���W
	int Dz; //�m�[�h�̖ړI�n��z���W
	double Sx; //�e�m�[�h��x�����̑��x
	double Sy; //�e�m�[�h��y�����̑��x
	double Sz; //�e�m�[�h��z�����̑��x
	int F = 0; //�e�m�[�h�̃t���O
	int ZF = 0; //�e�m�[�h��z���̃t���O
	int SP;
	int PS; //�ړ��J�n�ʒu����߂����_�ԍ�
	int PD; //�ړI�ʒu����߂����_�ԍ�
	int RootNo; //�m�[�h�̌������Ă��钸�_�̏��
};

struct Cost {
	int C = 0;
	int Ave = 0;
};

struct BanedAreas {
	double XA;
	double XB;
	double YA;
	double YB;
};

struct RoCo {
	int OnRoad = 0; //���H��ŃR���^�N�g
	int NoOnRoad = 0; //���H�ȊO�ŃR���^�N�g
};