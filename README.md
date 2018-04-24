## Biped Kinematics code ##

### Prerequisite

#### ・Ubuntu16.04
#### ・gnuplot ( Version 5.0.5 )

### ・install gnuplot-5.0.5
#### ・get gnuplot-5.0.5
```
wget http://sourceforge.net/projects/gnuplot/files/gnuplot/5.0.5/gnuplot-5.0.5.tar.gz
tar zxvf gnuplot-5.0.5.tar.gz
````
#### ・Additional installation package
````
sudo apt-get install libreadline6-dev
sudo apt-get install libgd2-xpm-dev libpango1.0-dev

````
#### ・build & install
```
cd gnuplot-5.0.5
./configure --with-readline=gnu
make -j 4
sudo make install
````
### ・Setting 
```
gnuplot
set terminal qt

````
### ・Test Smaple
```
cmake .
make 
make install

./biped
