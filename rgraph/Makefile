PRODUCT := region.so # rgraph.so

CXX := g++
LINKER := g++
CXXFLAGS := -Wall -Wextra -O0 -shared -lpython2.7 -lboost_python -lmpfr -lgmpxx -lgmp -fpic -g
INCDIRS := $(shell pkg-config --cflags python2)

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
