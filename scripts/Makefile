CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

all: opv-mod-fresh opv-mod-cpfsk opv-mod-hdl opv-sync-test

opv-mod-fresh: opv-mod-fresh.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

opv-mod-cpfsk: opv-mod-cpfsk.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

opv-mod-hdl: opv-mod-hdl.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

opv-sync-test: opv-sync-test.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

clean:
	rm -f opv-mod-fresh opv-mod-cpfsk opv-mod-hdl opv-sync-test *.iq

.PHONY: all clean
