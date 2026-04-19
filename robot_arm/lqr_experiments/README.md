# LQR Experiments — UR5e Stage 1

Experimentos cuantitativos del controlador LQR para la defensa de Etapa 1.

Corren de forma **headless** (sin Unity) contra el backend C++ en el puerto
`8848`. Cada ensayo abre una trayectoria minimum-jerk como referencia y después
cierra el lazo llamando a `/arm/step` en un bucle Python hasta `t = T`.

## Contenido

```
lqr_experiments/
├── run_experiments.py    # driver HTTP → CSV por corrida + summary.json
├── analyze.py            # CSV → figuras PNG/PDF + tabla xlsx
├── requirements.txt
└── results/              # se crea al ejecutar
    ├── *.csv
    ├── summary.json
    ├── summary.xlsx
    └── figures/
        ├── exp1_tracking.{png,pdf}
        ├── exp1_torques.{png,pdf}
        ├── exp2_errors.{png,pdf}
        ├── exp2_bars.{png,pdf}
        ├── exp2_bars_tradeoff.{png,pdf}
        ├── exp3_errors.{png,pdf}
        ├── exp3_bars.{png,pdf}
        └── exp3_bars_tradeoff.{png,pdf}
```

## Experimentos

| Código | Nombre                          | Qué varía                         | Configs |
|--------|---------------------------------|-----------------------------------|---------|
| Exp 1  | Tracking básico                 | nada (baseline)                   | 1       |
| Exp 2  | Barrido de pesos Q/R            | `wu ∈ {1.0, 0.3, 0.1, 0.03, 0.01}`| 5       |
| Exp 3  | Barrido de horizonte `N`        | `N ∈ {5, 10, 20, 40, 80}`         | 5       |

Los pesos base son `wq=30, wdq=2, wu=0.1, wqN=30, wdqN=2`. La trayectoria
por defecto es `q_start = 0`, `q_target = [0.8, -0.6, 0.7, -0.4, 0.5, -0.3]`,
`T = 1.5 s`, `dt = 0.02 s` (75 pasos por episodio).

## Cómo correrlo

### 1. Arrancar el backend C++

En una terminal aparte:

```bash
cd robot_arm/cmake-build-debug
./robot_arm_app          # o el binario que te genere CMake
```

Debe quedar escuchando en `http://127.0.0.1:8848`. Verifica con:

```bash
curl -X POST http://127.0.0.1:8848/arm/plan_minjerk_q \
  -H "Content-Type: application/json" \
  -d '{"q_target":[0,0,0,0,0,0],"T":0.5,"dt":0.1}'
```

Si devuelve un JSON con `trajectory`, todo OK.

### 2. Instalar dependencias Python

```bash
pip install -r lqr_experiments/requirements.txt
```

### 3. Correr los experimentos

```bash
cd lqr_experiments
python run_experiments.py
```

Salida esperada (aprox. 1–2 minutos, 11 configs × 75 pasos = 825 peticiones):

```
Running 11 LQR rollouts → .../results
▶ exp1_baseline  [Baseline (wq=30, wu=0.1, N=20)]
    rms_eq=0.0123 rad   u_energy=12.345   itae=0.0234   max|eq|=0.0567
▶ exp2_wu_very_soft  [wu=1]
    ...
Done. 11/11 runs in 43.2s
Summary written to .../results/summary.json
```

Si quieres correr solo un grupo:

```bash
python run_experiments.py --only exp2
```

### 4. Generar figuras y tabla

```bash
python analyze.py
```

Produce las PNG/PDF en `results/figures/` y `summary.xlsx` (o `summary.csv` si
no tienes `openpyxl`).

## Interpretación rápida (guía para la defensa)

- **Exp 1 — baseline**: muestra que el LQR sigue la trayectoria minimum-jerk
  con error RMS < 0.02 rad por junta. Plot `exp1_tracking.pdf` = dos columnas,
  izquierda `q` vs `q_ref` por junta, derecha el error.

- **Exp 2 — Q/R**: con `wu` chico (control barato → agresivo) el error baja
  pero la energía de τ crece. Con `wu` grande (control caro → suave) el brazo
  se queda atrás de la referencia. `exp2_bars_tradeoff.pdf` muestra la curva
  de Pareto.

- **Exp 3 — N**: `N = 5` muestra overshoot reactivo, `N = 80` casi no mejora
  respecto a `N = 20` pero cuesta más CPU. Moraleja: `N = 20` es suficiente
  para el horizonte `T = 1.5 s / dt = 0.02 = 75` pasos.

## Notas de diseño

- El script usa `q_cmd` devuelto por el backend como "medición" para el
  siguiente paso. Eso modela el lazo cerrado "planta integrada con la ley de
  control" dentro del propio backend.
- No hay ruido de medición ni incertidumbre paramétrica en estos
  experimentos (eso sería Exp 4, que dejamos fuera por alcance). Si lo
  quieres añadir más tarde, `use_kalman=True` + `meas_var` en el request ya
  están soportados por el backend.
- Para la defensa es recomendable ejecutar estos experimentos **una vez en
  frío** (guarda los PNG/PDF) y después presentar las figuras. No los corras
  en vivo frente al tribunal salvo que quieras demostrar que reproducen.
