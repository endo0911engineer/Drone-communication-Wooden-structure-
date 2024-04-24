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
効率のよい移動通信を実現するためにどのような仮想道路を作成すればよいのか、ドローンのスペックをどの程度にすればよいのか
がこの研究の神髄になります。
