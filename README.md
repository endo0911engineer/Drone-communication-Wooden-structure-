# Drone-communication-Wooden-structure-
大学時代に行っていたドローンの移動通信の研究になります。

## 使用言語
C++,C

## 実行環境
ubuntu(バージョンは覚えていません)

## 実行方法
C/C++言語のコンパイラがインストールされている環境で<br><br>
g++ main.cpp Connection.cpp CreateVertex.cpp NodeInfo.cpp<br><br>
を実行する。<br>
実行後 NumberOfContactAndMovingCost.txt ContactArea.txt ファイルが作成される。<br>
NumberOfContactAndMovingCost.txtにはドローン同士の総コンタクト回数が記録され、ContactArea.txtにはドローンがコンタクトした場所が記録される。

## プロジェクトの概要
ドローンの移動通信について研究したプログラムになります。<br>
上空にドローン専用の仮想道路を作成し、複数のドローンが仮想道路上を移動します。<br>
ドローンには通信可能範囲が決められており、互いがその通信可能範囲ですれ違うことが出来ればコンタクト1回となります。<br>
一定時間複数のドローンが仮想道路を移動し、より少ない迂回距離でより多くのコンタクト回数が得られれば効率のよい通信となります。<br>
効率のよい移動通信を実現するためにどのような仮想道路を作成すればよいのか、ドローンのスペックをどの程度にすればよいのかがこの研究の神髄になります。<br><br>
実際の研究では複数の仮想道路を作成し比較を行います。<br>
今回のプログラムでは木構造の仮想道路を採用しています。

## プログラム詳細
### connection.cpp
ドローン同士の通信を行う処理が記述されたファイルである。<br>
- CalculateDistanceBetweenTwoPointsでドローン同士の距離を計算する。
- ConnectTwoPointsでドローン同士の距離から通信可能かどうかを判断し、通信回数と通信した場所を記録する。
### CreateVertex.cpp
ドローンが上空を飛行する際の仮想道路を作成する際に必要な処理が記述されたファイルである。<br>
- CreateVertexで飛行禁止エリアを考慮して頂点と辺の生成を行う。
- CreateBanedAreasで飛行禁止エリアを生成する。
- CreateVertexで頂点をランダムに生成する。
- SortVertexで中心に近い順で反時計回りに頂点のソートを行う。
- CreateEdgeで飛行禁止エリアを考慮し頂点同士を辺でつなげる。
- DijkstraMethodで最も中心に近い頂点から各頂点への最短距離を求める。
- DFSで中心に近い頂点から深さ優先探索を行いグラフが成立しているかどうかを確かめる。
### DataBase.h
コンピュータシミュレーションを行う上でのドローンのスペックや仮想道路の情報の定義、それぞれの変数宣言などを行う。
### Nodeinfo.cpp
ドローンが生成され、地上から上昇し上空の仮想道路を移動し目的地まで下降する一連の流れの処理を行う。<br>
- GenerateNodeでドローンを生成する。
- DestinationAndSpeedGenerationでドローンの目的地とスピードが与えられる。
- DACostでドローンがダイクストラ法に従って目的地にたどり着いたときの迂回距離を計算する。
- DAでドローンの目的地までの最短距離を求める。
- Movingでドローンを移動させる。
- MoveToでドローンの進行方向を決定する。 
### main.cpp
それぞれのファイルから関数を呼び出し、一定時間ドローンを飛行させ、総コンタクト回数とコンタクト場所情報を格納したファイルが生成する。<br>
- GraphCheckによりグラフが生成されGenerateNodeによりドローンが地上に生成される。
- DestinationAndSpeedGenerationによりドローンの目的地とスピードが生成されMovingでドローンが移動する。
- CalculateDistanceBetweenTwoPointsでドローン同士の距離を計算しConnectTwoPointsでドローンの通信回数と通信した場所を記録する。
- これらの一連の流れを一定時間繰り返す。
