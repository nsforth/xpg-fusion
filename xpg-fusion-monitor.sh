#!/bin/bash
# xpg-fusion-monitor.sh — live telemetry for the XPG Fusion 1600W PSU.
# Reads the xpg-fusion hwmon (kernel module); no root required.
# Usage: ./xpg-fusion-monitor.sh [interval-seconds]
set -uo pipefail
export LC_ALL=C

interval="${1:-1}"
[[ "$interval" =~ ^[0-9]+$ && "$interval" -ge 1 ]] || {
    echo "Usage: $0 [interval-seconds]"; exit 1;
}

resolve_h() {
    local f
    f=$(grep -l xpg_fusion /sys/class/hwmon/hwmon*/name 2>/dev/null | head -1)
    [ -n "$f" ] && h="${f%/*}" || h=""
}

resolve_h
if [ -z "$h" ]; then
    echo "ERROR: xpg_fusion hwmon not found (is the module loaded? lsmod | grep xpg)" >&2
    exit 1
fi

w()   { [ "${1:-NA}" = NA ] && printf '    NA' || awk -v x="$1" 'BEGIN{printf "%6.1f", x/1e6}'; }
t()   { [ "${1:-NA}" = NA ] && printf "    NA" || awk -v x="$1" 'BEGIN{printf "%5.1f", x/1000}'; }
r()   { cat "$h/$1" 2>/dev/null || echo NA; }

printf "PSU: %s   FW: %s\n" "$(tr -dc '[:print:]' < "$h/model" 2>/dev/null)" "$(tr -dc '[:print:]' < "$h/firmware_version" 2>/dev/null)"
printf "%-8s %8s %8s %5s %5s %7s %7s %8s\n" "time" "DC-out" "AC-in" "eff%" "psu%" "GaN" "12Vtot" "fan"
while :; do
    # hwmonN indices are recycled — re-resolve the directory every iteration
    resolve_h || true
    if [ -z "$h" ]; then
        echo "ERROR: xpg_fusion hwmon disappeared (module unloaded?)" >&2
        exit 1
    fi
    pout=$(r power1_input); pin=$(r power2_input)
    temp=$(r temp1_input);  fan=$(r fan1_input)
    psu=$(r efficiency_percent)
    # +12V total current as measured by the PSU (curr11)
    rails=$(r curr11_input); ok=1
    [ "$rails" = NA ] && ok=0
    if [ "$pout" != NA ] && [ "$pin" != NA ] && [ "$pin" -gt 0 ] 2>/dev/null; then
        eff=$(awk -v a="$pout" -v b="$pin" 'BEGIN{printf "%3.0f", a*100/b}')
    else
        eff=NA
    fi
    if [ "$ok" = 1 ]; then
        rails_s=$(awk -v x="$rails" 'BEGIN{printf "%5.1f A", x/1000}')
    else
        rails_s='    NA'
    fi
    fan_s="$([ "${fan:-NA}" != NA ] && echo "$fan rpm" || echo '    NA')"
    # efficiency_percent is cached by the driver even on lost contact —
    # show it only while the channel itself is alive
    if [ "$pout" != NA ] && [ "$psu" != NA ]; then psu_s="$psu"; else psu_s="NA"; fi
    printf "%-8s %8s %8s %5s %5s %7s %7s %8s\n" \
        "$(date +%H:%M:%S)" "$(w "$pout") W" "$(w "$pin") W" "$eff" \
        "$psu_s" "$(t "$temp") C" "$rails_s" "$fan_s"
    sleep "$interval"
done
