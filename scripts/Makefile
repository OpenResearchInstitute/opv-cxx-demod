CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

all: opv-mod-fresh opv-mod-cpfsk opv-mod-hdl opv-sync-test opv-demod-full opv-demod-afc

opv-mod-fresh: opv-mod-fresh.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

opv-mod-cpfsk: opv-mod-cpfsk.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

opv-mod-hdl: opv-mod-hdl.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

opv-sync-test: opv-sync-test.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

opv-demod-full: opv-demod-full.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

opv-demod-afc: opv-demod-afc.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

clean:
	rm -f opv-mod-fresh opv-mod-cpfsk opv-mod-hdl opv-sync-test opv-demod-full opv-demod-afc

.PHONY: all clean
