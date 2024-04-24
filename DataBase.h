#pragma once
#include <vector>
#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <fstream>
#include <iostream>

using namespace std;

#define NODE 2 //ノードの数
#define MIN_SPEED 30 //ノードの最高速度
#define MAX_SPEED 30 //ノードの最低速度
#define Z_SPEED 10 //z軸の速度
#define RANGE 100 //通信可能距離
#define NUMBER_OF_VERTEX 25 //頂点の数
#define SERVICE_AREA_X 1000 //サービスエリアのx方向の範囲
#define SERVICE_AREA_Y 1000 //サービスエリアのy方向の範囲
#define HIGHT_OF_ROAD 1000 //上空の仮想道路の高さ
#define NOBA 2 //NUMBER_OF_BANED_AREAS(飛行禁止エリアの数)

struct Vertex {
	int Vx; //頂点のx座標
	int Vy; //頂点のy座標
	double DisCen; //中央からの各頂点への距離とシータの値
};

struct Edge {
	int to; //隣接ノード
	double cost; //隣接ノードへのcost
};

struct Dijkstra {
	int VertexBeforeMoving;
	double TotalCost;
	int Check;
};

struct Node {
	double Cx; //ノードのx座標
	double Cy; //ノードのy座標
	double Cz; //ノードのz座標
	int Dx; //ノードの目的地のx座標
	int Dy; //ノードの目的地のy座標
	int Dz; //ノードの目的地のz座標
	double Sx; //各ノードのx方向の速度
	double Sy; //各ノードのy方向の速度
	double Sz; //各ノードのz方向の速度
	int F = 0; //各ノードのフラグ
	int ZF = 0; //各ノードのz軸のフラグ
	int SP;
	int PS; //移動開始位置から近い頂点番号
	int PD; //目的位置から近い頂点番号
	int RootNo; //ノードの向かっている頂点の情報
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
	int OnRoad = 0; //道路上でコンタクト
	int NoOnRoad = 0; //道路以外でコンタクト
};