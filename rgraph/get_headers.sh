#!/usr/bin/env bash
wget https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-5.0.2/CGAL-5.0.2.tar.xz
wget https://dl.bintray.com/boostorg/release/1.73.0/source/boost_1_73_0.tar.gz

tar -xf CGAL-5.0.2.tar.xz
tar -xf boost_1_73_0.tar.gz
cd boost_1_73_0
./bootstrap.sh --with-python=python2 --with-libraries=python
./b2
