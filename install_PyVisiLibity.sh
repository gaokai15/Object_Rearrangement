#!/usr/bin/env bash
git clone https://github.com/tsaoyu/PyVisiLibity.git
patch -ruN -d PyVisiLibity < silence.patch
pip install ./PyVisiLibity
