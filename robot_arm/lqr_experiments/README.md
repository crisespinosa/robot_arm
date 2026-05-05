# LQR Experiments — UR5e (Etapa 1)

Experimentos cuantitativos del controlador **LQR de horizonte finito** para la
defensa de la etapa 1 del proyecto. No usan Unity, no dependen de ninguna
heurística externa: son **rollouts headless contra el backend C++** en el
puerto `8848`.

Cada corrida:

1. `POST /arm/set_reference` → el backend planifica y guarda la trayectoria
   minimum-jerk de referencia (polinomio quíntico, fórmula cerrada).
2. Bucle Python `POST /arm/step` a ritmo `dt` hasta `t = T`. En cada paso
   el backend devuelve `q_cmd`, `dq_cmd`, `tau_cmd`, `q_ref`, `dq_ref`,
   `ddq_ref`. El script guarda todo en CSV.
3. Al final se agregan métricas (RMS, energía, ITAE, máximo error, error
   final) en `summary.json`.

## Contenido

```
lqr_experiments/
├── run_experiments.py    # driver HTTP → CSV por corrida + summary.json
├── analyze.py            # CSV → figuras PNG/PDF + tabla xlsx
├── requirements.txt
└── results/              # se crea al ejecutar
    ├── *.csv             # una fila por paso, 6 articulaciones
    ├── summary.json      # métricas agregadas
    ├── summary.xlsx      # tabla resumen (fallback a summary.csv)
    └── figures/
        ├── exp1_tracking.{png,pdf}
        ├── exp1_torques.{png,pdf}
        ├── exp2_errors.{png,pdf}
        ├── exp2_bars.{png,pdf}
        ├── exp2_bars_tradeoff.{png,pdf}
        ├── exp3_errors.{png,pdf}
        ├── exp3_bars.{png,pdf}
        ├── exp3_bars_tradeoff.{png,pdf}
        ├── exp4_tracking_A_moderado.{png,pdf}
        ├── exp4_tracking_B_asimetrico.{png,pdf}
        ├── exp4_tracking_C_amplio.{png,pdf}
        ├── exp4_errors.{png,pdf}
        └── exp4_bars.{png,pdf}
```

## Experimentos

| Código | Nombre                          | Qué varía                           | Configs |
|--------|---------------------------------|-------------------------------------|---------|
| Exp 1  | Tracking baseline               | nada                                | 1       |
| Exp 2  | Barrido Q/R (`wu`)              | `wu ∈ {1.0, 0.3, 0.1, 0.03, 0.01}`  | 5       |
| Exp 3  | Barrido horizonte `N`           | `N ∈ {5, 10, 20, 40, 80}`           | 5       |
| Exp 4  | Generalización `q_target`       | 3 objetivos (moderado / asim. / amplio) | 3   |

Total: **14 rollouts**. En una máquina decente tarda ~1 min 15 s.

Configuración fija en los experimentos 1–3 (en Exp 4 sólo cambia `q_target`):

- `q_start = [0, 0, 0, 0, 0, 0]`
- `q_target = [0.8, -0.6, 0.7, -0.4, 0.5, -0.3]` en Exp 1/2/3 (6 DOF activos).
  En Exp 4 se prueban tres `q_target` distintos (ver abajo).
- `T = 1.5 s`, `dt = 0.02 s` → 75 pasos por episodio
- Pesos baseline: `wq=30, wdq=2, wu=0.1, wqN=30, wdqN=2`
- `u_max = 8 rad/s²`, `mode = "lqr"`, `use_kalman = false`
- Sin ruido de medición ni randomización de dinámica

`q_target` usados en Exp 4 (todos con los mismos pesos y `N=20`):

| Tag              | Descripción  | Vector                                  |
|------------------|--------------|-----------------------------------------|
| A (moderado)     | medio recorrido, simétrico   | `[0.4, -0.3, 0.4, -0.2, 0.3, -0.15]`    |
| B (asimétrico)   | 3 DOF bloqueados, 3 forzados | `[1.0,  0.0, 0.8,  0.0, 0.4,  0.0 ]`    |
| C (amplio)       | 1.5× baseline, estresa `u_max` | `[1.2, -0.9, 1.0, -0.7, 0.8, -0.5 ]`  |

---

## Requisitos (Ubuntu 20.04 / 22.04 / 24.04)

El backend está en C++ (Drogon). Los scripts de experimentos son Python 3.

```bash
# Herramientas de compilación (solo si aún no compilaste el backend)
sudo apt update
sudo apt install -y build-essential cmake git pkg-config curl

# Python 3 con venv (recomendado en Ubuntu moderno)
sudo apt install -y python3 python3-venv python3-pip
```

Versiones mínimas: Python 3.9+, CMake 3.18+, gcc 9+.

---

## Cómo correrlo — paso a paso

El flujo tiene **tres terminales** abiertas (o una con `tmux`, como prefieras):

- Terminal A — corre el backend C++.
- Terminal B — corre los experimentos Python.
- Terminal C — (opcional) verificaciones puntuales con `curl`.

### 1. Compilar y arrancar el backend — Terminal A

Desde la raíz del repo:

```bash
cd robot_arm
mkdir -p cmake-build-release
cd cmake-build-release
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j"$(nproc)"
```

Esto produce el binario `./robot_arm_app` (el nombre exacto depende de tu
`CMakeLists.txt`; si es distinto, sustitúyelo en los comandos siguientes).

Arranca el servidor:

```bash
./robot_arm_app
```

Debe quedar escuchando en `http://127.0.0.1:8848`. Déjalo corriendo en esa
terminal. Para detenerlo, `Ctrl+C`.

> **Nota.** Si compilaste con `cmake -DCMAKE_BUILD_TYPE=Debug` el binario
> estará en `cmake-build-debug/` en vez de `cmake-build-release/`. Funciona
> igual, sólo es más lento. Para la defensa usa Release.

### 2. Verificar que el backend responde — Terminal C (opcional)

Un ping rápido con `curl`:

```bash
curl -X POST http://127.0.0.1:8848/arm/plan_minjerk_q \
  -H "Content-Type: application/json" \
  -d '{"q_target":[0,0,0,0,0,0],"T":0.5,"dt":0.1}'
```

Si devuelve un JSON con un campo `"trajectory"` de 6 puntos, todo OK.
Si responde «Connection refused», el backend no está corriendo: vuelve
al paso 1.

### 3. Preparar el entorno Python — Terminal B

Desde la raíz del repo:

```bash
cd robot_arm/lqr_experiments

# Entorno virtual (recomendado en Ubuntu 22.04+ por PEP 668)
python3 -m venv .venv
source .venv/bin/activate

pip install --upgrade pip
pip install -r requirements.txt
```

Dependencias (ver `requirements.txt`):

- `requests` — cliente HTTP contra el backend.
- `numpy`, `pandas` — métricas.
- `matplotlib` — figuras.
- `openpyxl` — `summary.xlsx`.

Si en futuros `cd` pierdes el venv, reactívalo con:

```bash
source /ruta/a/robot_arm/lqr_experiments/.venv/bin/activate
```

### 4. Correr los experimentos — Terminal B

Con el venv activo y el backend corriendo en Terminal A:

```bash
python run_experiments.py
```

Salida esperada (aprox. 1 min 15 s):

```
Running 14 LQR rollouts → .../results
▶ exp1_baseline  [Baseline (wq=30, wu=0.1, N=20)]
    rms_eq=0.0123 rad   u_energy=12.345   itae=0.0234   max|eq|=0.0567
▶ exp2_wu_very_soft  [wu=1]
    ...
▶ exp4_target_A_moderado  [Target A (moderado)]
    ...
Done. 14/14 runs in 68.4s
Summary written to .../results/summary.json
Next: python analyze.py
```

Variantes útiles:

```bash
# Solo un grupo de experimentos
python run_experiments.py --only exp1
python run_experiments.py --only exp2
python run_experiments.py --only exp3
python run_experiments.py --only exp4

# Backend en otro puerto/host
python run_experiments.py --backend http://192.168.1.42:8848

# Escribir los CSV en otra carpeta
python run_experiments.py --results-dir /tmp/lqr_run_01
```

### 5. Generar figuras y tabla — Terminal B

```bash
python analyze.py
```

Produce (en `results/figures/`):

- `exp1_tracking.{png,pdf}` — 6 articulaciones, `q(t)` vs `q_ref(t)` y
  error en columna adyacente.
- `exp1_torques.{png,pdf}` — perfil de torques de las 6 articulaciones.
- `exp2_errors.{png,pdf}` — error por articulación para los 5 valores de `wu`.
- `exp2_bars.{png,pdf}` — barras RMS y energía por configuración.
- `exp2_bars_tradeoff.{png,pdf}` — scatter RMS vs energía (curva de Pareto).
- `exp3_errors.{png,pdf}` — error por articulación para los 5 horizontes.
- `exp3_bars.{png,pdf}` y `exp3_bars_tradeoff.{png,pdf}` — ídem para `N`.
- `exp4_tracking_A_moderado.{png,pdf}`, `exp4_tracking_B_asimetrico.{png,pdf}`,
  `exp4_tracking_C_amplio.{png,pdf}` — tracking completo (q vs q_ref + error)
  por cada uno de los 3 objetivos.
- `exp4_errors.{png,pdf}` — error por articulación superpuesto para los 3 targets.
- `exp4_bars.{png,pdf}` — 2×2 barras con `rms_eq`, `max|eq|`, `eq_final` y
  `∫‖τ‖²` por target.

Y en `results/`:

- `summary.xlsx` (o `summary.csv` si falta `openpyxl`) — una fila por
  corrida con todas las métricas.

Para verlas desde Ubuntu:

```bash
# PDF con el visor por defecto
xdg-open results/figures/exp1_tracking.pdf

# PNG
xdg-open results/figures/exp1_tracking.png

# Hoja de cálculo
xdg-open results/summary.xlsx
```

---

## Interpretación (guía para la defensa)

**Exp 1 — baseline.** Demuestra que el LQR sigue la referencia minimum-jerk
con RMS < 0.02 rad por articulación. `exp1_tracking.pdf` es tu figura
principal: dos columnas, izquierda `q` vs `q_ref`, derecha el error.

**Exp 2 — Q/R (pesos).** Con `wu` chico el control es «barato» → el brazo
reacciona agresivo, baja el error pero sube `∫‖τ‖²`. Con `wu` grande el
control es «caro» → el brazo se queda atrás de la referencia. Tu valor
baseline (`wu=0.1`) está cerca del codo de la curva. El gráfico clave es
`exp2_bars_tradeoff.pdf`: muestra la curva de Pareto.

**Exp 3 — horizonte `N`.** `N=5` es miope y produce overshoot reactivo.
`N=20` ya converge. `N=80` no mejora respecto a `N=20` pero cuesta más CPU.
Conclusión: `N=20` es el mínimo suficiente para `T=1.5 s / dt=0.02`.

**Exp 4 — generalización.** Con los **mismos pesos y el mismo horizonte**,
el regulador sigue bien tres objetivos distintos: uno moderado (recorrido
medio), uno asimétrico (la mitad de los DOF bloqueados) y uno amplio (1.5×
baseline, que estresa `u_max=8 rad/s²`). El mensaje es que el LQR no está
sintonizado para un único `q_target`: la misma ley de control funciona en
distintos escenarios de posicionamiento. El gráfico clave es `exp4_bars.pdf`
(4 métricas en 2×2): `rms_eq`, `max|eq|` y `eq_final` se mantienen en el
mismo orden de magnitud; la energía `∫‖τ‖²` crece con la amplitud, como es
esperable.

Los cuatro experimentos son independientes y se pueden discutir por separado.

---

## Troubleshooting en Ubuntu

| Síntoma | Causa probable | Solución |
|---------|----------------|----------|
| `Connection refused` al hacer `curl` o al correr `run_experiments.py` | El backend no está arrancado | Abre Terminal A y arranca `./robot_arm_app` |
| `error: externally-managed-environment` al hacer `pip install` | Ubuntu 22.04+ con PEP 668 | Usa un venv (paso 3) o añade `--break-system-packages` (no recomendado) |
| El backend arranca pero `curl` devuelve HTML de error o 404 | Endpoint desactualizado o puerto ocupado | Verifica con `ss -ltnp \| grep 8848` que tu binario sea el que está escuchando |
| `ModuleNotFoundError: No module named 'requests'` | Olvidaste activar el venv | `source .venv/bin/activate` |
| `xdg-open` no abre el PDF | Falta un visor por defecto | `sudo apt install evince` o abre con `evince results/figures/exp1_tracking.pdf` directamente |
| El `summary.xlsx` no se genera, sale `summary.csv` | Falta `openpyxl` | `pip install openpyxl` dentro del venv |
| Matplotlib falla con `Could not connect to X display` | `analyze.py` intenta renderizar en pantalla | Ya está parcheado con `matplotlib.use("Agg")` al inicio del script; si ves esto, asegúrate de no haberlo comentado |

---

## Notas de diseño

- El script usa `q_cmd` y `dq_cmd` devueltos por el backend como «medición»
  del siguiente paso. Eso modela el lazo cerrado «planta integrada con la
  ley de control» dentro del propio backend. Ningún integrador separado
  en Python.
- No hay ruido de medición ni incertidumbre paramétrica (`inertia_scale=1`,
  `friction_scale=1`). Un experimento de robustez (Kalman + ruido) sería el
  siguiente paso, pero queda **fuera de alcance de la defensa de esta
  etapa** para no diluir el mensaje «la base es LQR».
- Recomendación para la defensa: corre estos experimentos **una vez en
  frío** en tu máquina, guarda los PNG/PDF en una carpeta aparte con fecha,
  y en la defensa sólo presenta las figuras. No los corras en vivo frente
  al tribunal salvo que quieras demostrar reproducibilidad.

---

## Qué *no* está aquí (explícito)

- No hay resolución numérica de PMP. La trayectoria de referencia se
  calcula con la fórmula cerrada del quíntico (que *coincide* con la
  solución óptima por PMP para el caso minimum-jerk, pero no se resuelve
  PMP como problema de optimización).
- No hay integración con Unity. Unity se usa sólo como demo visual
  separada; los números de la defensa salen de este pipeline headless.
