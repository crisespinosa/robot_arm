#!/usr/bin/env bash
# ============================================================
# test_episode.sh — Smoke test del backend robot_arm.
#
# Lanza 1 episodio completo (set_reference + 75 pasos) contra el
# backend en 127.0.0.1:8848 para que veas en la terminal del
# backend los logs [set_reference], [step] cada 10 pasos, y al
# final el resumen ========== EPISODE METRICS ==========.
#
# Uso:
#   1) En una terminal:    ./robot_arm     (desde build/)
#   2) En otra terminal:   ./test_episode.sh
# ============================================================

set -euo pipefail

URL=${URL:-"http://127.0.0.1:8848"}
T=${T:-1.5}
DT=${DT:-0.02}
Q_TARGET=${Q_TARGET:-"[0.8,-0.6,0.7,-0.4,0.5,-0.3]"}

echo "1) POST /arm/set_reference  (T=${T}s  dt=${DT}s)"
curl -sS -X POST "${URL}/arm/set_reference" \
    -H "Content-Type: application/json" \
    -d "{\"q_target\":${Q_TARGET},\"T\":${T},\"dt\":${DT}}" | head -c 200
echo

# Numero de pasos = round(T/DT)
N_STEPS=$(awk -v T="${T}" -v DT="${DT}" 'BEGIN{printf "%d", T/DT + 0.5}')
echo
echo "2) Loop /arm/step  (${N_STEPS} pasos)"

for ((k=0; k<=N_STEPS; k++)); do
    t=$(awk -v k="${k}" -v dt="${DT}" 'BEGIN{printf "%.4f", k*dt}')
    curl -sS -X POST "${URL}/arm/step" \
        -H "Content-Type: application/json" \
        -d "{\"q\":[0,0,0,0,0,0],\"dq\":[0,0,0,0,0,0],\"t\":${t},\"dt\":${DT},\"mode\":\"lqr\",\"weights\":{\"wq\":40,\"wdq\":8,\"wu\":0.4,\"wqN\":40,\"wdqN\":8},\"N\":20}" \
        > /dev/null
done

echo "3) Listo. Mira la terminal del backend."
echo "   Deberias ver:"
echo "     [set_reference] T=${T}s  dt=${DT}s ..."
echo "     [step] k=10 ..."
echo "     [step] k=20 ..."
echo "     ..."
echo "     ========== EPISODE METRICS =========="
