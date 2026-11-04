#!/bin/sh
# Flash/RAM/stack estimate of the firmware core from its -Os object files.
# These are host (arm64 Mach-O) objects, so this is an estimate: Thumb-2 code
# for a Cortex-M is usually smaller.
set -e
dir="$1"
objs=$(ls "$dir"/src/*.o)

echo
echo "== Firmware core footprint (static estimate from $dir/src/*.o, host arm64 -Os)"
size -m $objs | awk '
  /^[^ \t].*\.o:$/ { f = $1; sub(/:$/, "", f); n = split(f, p, "/"); name = p[n]; files[++nf] = name; next }
  /Section \(/ {
    match($0, /\(__[A-Z]+, [_a-z0-9]+\)/); sec = substr($0, RSTART, RLENGTH); v = $NF
    if (sec ~ /__TEXT, __text/)                         { code[name] += v; tc += v }
    else if (sec ~ /__TEXT, (__const|__cstring|__literal)/ || sec ~ /__DATA, __const/) { ro[name] += v; tr += v }
    else if (sec ~ /__DATA, __data/)                    { data[name] += v; td += v }
    else if (sec ~ /__DATA, (__bss|__common)/)          { bss[name] += v; tb += v }
  }
  END {
    printf "  %-14s %8s %8s %8s %8s\n", "object", "code", "rodata", "data", "bss"
    for (i = 1; i <= nf; i++) { k = files[i]; printf "  %-14s %8d %8d %8d %8d\n", k, code[k], ro[k], data[k], bss[k] }
    printf "  %-14s %8d %8d %8d %8d\n", "total", tc, tr, td, tb
    printf "flash estimate (code + rodata + data): %d B\n", tc + tr + td
    printf "static RAM in .data/.bss: %d B (firmware state lives in the Firmware instance, see sizeof above)\n", td + tb
  }'
echo "deepest single stack frames (-fstack-usage; call-chain depth not analyzed):"
cat "$dir"/src/*.su | sort -t"$(printf '\t')" -k2 -n -r | head -5 | awk -F'\t' '{ n = split($1, p, ":"); printf "  %6d B  %s\n", $2, p[n] }'
