CXX = g++
CXXFLAGS = -std=c++17 -O3 -Wall
BINDIR = bin

all: $(BINDIR)/opv-mod $(BINDIR)/opv-demod

$(BINDIR):
	mkdir -p $(BINDIR)

$(BINDIR)/opv-mod: src/opv-mod.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

$(BINDIR)/opv-demod: src/opv-demod.cpp | $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $< -lm

clean:
	rm -rf $(BINDIR)

test: all
	@$(BINDIR)/opv-mod -S TEST -B 5 | $(BINDIR)/opv-demod -s 2>&1 | grep -E "Station|Token|Summary"

.PHONY: all clean test
