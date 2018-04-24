## Biped Kinematics code ##

### 動作環境

#### ・Ubuntu16.04
#### ・gnuplot ( Version 5.0.5 )
#### ・cmake version 2.8.12.2

###　セットアップ
#### ・Install
1.必要なパッケージのインストールを行う.( cmake は略. バージョン2.8.12を入れてください )
##### get boost & eigen 
```
sudo apt-get update
sudo apt install libboost-all-dev libeigen3-dev
````
##### ・get gnuplot-5.0.5
```
wget http://sourceforge.net/projects/gnuplot/files/gnuplot/5.0.5/gnuplot-5.0.5.tar.gz
tar zxvf gnuplot-5.0.5.tar.gz
````
###### ・build & install
```
cd gnuplot-5.0.5
./configure --with-readline=gnu
make -j 4
sudo make install
````
#### 環境変数の設定
 /etc/environmentに必要な環境変数を追加
````
sudo vi /etc/environment
````
内部に以下の行を追加し保存
````
EIGEN3_INCLUDE_DIR="/usr/include/eigen3"
````
Ubuntuを再起動
````
sudo reboot
````

#### ・Setting 
```
gnuplot
set terminal qt

````
#### ・Test Smaple
```
cmake .
make 
make install

./biped
