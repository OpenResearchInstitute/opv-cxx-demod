#!/usr/bin/env bash
set -euo pipefail

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

make bin/opv-mod bin/opv-demod

./bin/opv-mod -S W5NYV -P -B 5 > "$tmp/signal.iq"

./bin/opv-demod -s -r -q \
    < "$tmp/signal.iq" \
    > "$tmp/recovered.bin" \
    2> "$tmp/demod.log"

perl -e '
use strict;
use warnings;

my ($iq_file, $rx_file) = @ARGV;

my $iq_size = -s $iq_file;
die "FAIL: I/Q size is $iq_size\n"
    unless $iq_size == 2097280;

open my $fh, "<:raw", $rx_file or die $!;
local $/;
my $actual = <$fh>;

my $header = pack("H*", "000003742697BBAADD000000");

my $expected = join "", map {
    my $i = $_;
    $header . pack("C*", map { ($i + $_) & 255 } 0..121)
} 0..4;

die "FAIL: Recovered frame data mismatch\n"
    unless $actual eq $expected;

print "PASS: I/Q output size is correct\n";
print "PASS: All 5 frames match byte-for-byte\n";
print "PASS: Windows binary I/O regression test\n";
' "$tmp/signal.iq" "$tmp/recovered.bin"

# Verify raw-mode binary input and recovered output.
./bin/opv-mod -P -R \
    < "$tmp/recovered.bin" \
    > "$tmp/raw.iq"

./bin/opv-demod -s -r -q \
    < "$tmp/raw.iq" \
    > "$tmp/raw-recovered.bin" \
    2> "$tmp/raw-demod.log"

perl -e '
use strict;
use warnings;

my ($src, $dst) = @ARGV;

open my $a, "<:raw", $src or die $!;
open my $b, "<:raw", $dst or die $!;

local $/;
my $original = <$a>;
my $recovered = <$b>;

die "FAIL: Raw-mode binary mismatch\n"
    unless $original eq $recovered;

print "PASS: Raw-mode binary round-trip\n";
' "$tmp/recovered.bin" "$tmp/raw-recovered.bin"
