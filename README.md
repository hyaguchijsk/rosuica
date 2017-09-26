# rosuica

## 著者: 矢口 裕明

## 概要

このパッケージはSUICAの利用履歴を読み取り，topicとして出力します．

## ハードウェア

Sony RC-S380で動作確認

## 使用ライブラリ

nfcpy

## 使用方法

まず[nfcpy](https://nfcpy.readthedocs.io/en/latest/index.html)をインストールしてください．

```
$ sudo pip install -U nfcpy
```

本リポジトリを`{catkin_ws}/src`以下に展開してください．

```
$ wstool set --git rosuica https://github.com/hyaguchijsk/rosuica.git
$ wstool update rosuica
```

rosuicaパッケージをビルドしてパスを通してください．

```
$ catkin build rosuica
$
```

NFCリーダーを接続し，認識させてください．
この操作は機種によります．

ノードを立ち上げてください．

```
$ rosrun rosuica suica.py
```

### トピックについて

`/suica_history`に`std_msgs/Int32MultiArray`で出力します．

現在は直近20件の取引後の残高を出力します．
