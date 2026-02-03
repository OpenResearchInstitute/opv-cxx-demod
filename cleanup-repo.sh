#!/bin/bash
# OPV Repository Cleanup Script
# Run from: /Users/w5nyv/opv-cxx-demod
#
# BEFORE RUNNING: Create the legacy branch first!
#   git checkout -b legacy-cmake-structure
#   git push origin legacy-cmake-structure
#   git checkout main

set -e

echo "=== OPV Repository Cleanup ==="

# Safety check
if [ ! -d ".git" ]; then
    echo "ERROR: Not in a git repository!"
    exit 1
fi

echo "Step 1: Remove obsolete directories..."
rm -rf apps/ build/ cmake/ example/ include/ src/ tests/ grc/

echo "Step 2: Remove obsolete root files..."
rm -f CMakeLists.txt win_build.bat commit.txt

echo "Step 3: Clean up scripts directory..."
cd scripts/
rm -f opv-demod-afc-old.cpp opv-demod-full.cpp opv-demod-full
rm -f opv-mod-hdl-old.cpp opv-mod-fresh.cpp opv-mod-fresh
rm -f opv-mod-cpfsk.cpp opv-mod-cpfsk
rm -f opv-sync-test.cpp opv-sync-test run-sync-test.sh
rm -f opv-pluto-rx-old.sh
rm -f *.log test.iq opv-demod-afc opv-mod-hdl
mv opv-demod-afc.cpp opv-demod.cpp 2>/dev/null || true
mv opv-mod-hdl.cpp opv-mod.cpp 2>/dev/null || true
cd ..

echo "Step 4: Reorganize..."
mkdir -p src docs
mv scripts/opv-demod.cpp src/ 2>/dev/null || true
mv scripts/opv-mod.cpp src/ 2>/dev/null || true
mv doc/* docs/ 2>/dev/null || true
rm -rf doc/
mv filter-taps.ipynb docs/ 2>/dev/null || true
mv numerology.ipynb docs/ 2>/dev/null || true
rm -f scripts/Makefile scripts/Readme.md scripts/ESG-upload.py
rm -f scripts/examine-*.ipynb

echo "Step 5: Create new Makefile..."
cat > Makefile << 'MAKEFILEEOF'
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
MAKEFILEEOF

echo "Step 6: Update .gitignore..."
cat > .gitignore << 'EOF'
bin/
*.o
.DS_Store
*.swp
*.iq
*.log
EOF

echo ""
echo "=== Done! ==="
echo ""
echo "Final structure:"
find . -type f -not -path './.git/*' | sort
echo ""
echo "Next: git add -A && git commit -m 'Simplify to standalone mod/demod' && git push"
