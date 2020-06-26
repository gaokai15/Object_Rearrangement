PRODUCT := region.so

CXX := g++
LINKER := g++
# CXXFLAGS := -Wall -Wextra -O3 -shared -lpython2.7 -lboost_python -lmpfr -lgmpxx -lgmp -frounding-math -fpic -g -pg
CXXFLAGS := -Wall -Wextra -O3 -shared -lpython2.7 -lboost_python -lmpfr -lgmpxx -lgmp -frounding-math -fpic
INCDIRS := $(shell pkg-config --cflags python2) -I$(shell pwd)/CGAL-5.0.2/include/

SRCDIR := ./src
SRCFILES := $(wildcard $(SRCDIR)/*.cpp)
OBJFILES := $(patsubst %.cpp,%.o,$(SRCFILES))

all: $(PRODUCT)

%.so: $(SRCDIR)/%.o
	$(LINKER) $(CXXFLAGS) $< -o $@ # $^ -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCDIRS) -c $< -o $@

.PHONY: clean
clean:
	rm -f $(PRODUCT) $(OBJFILES)
