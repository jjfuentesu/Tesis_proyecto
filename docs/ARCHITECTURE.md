# Documentación de Arquitectura y Etapas del Desarrollo

## Visión General del Proyecto
Este proyecto implementa el sistema de software para la investigación doctoral comparativa entre un controlador PID clásico y un **Fuzzy Self-Tuning PID** sobre un dron educativo (DJI Tello EDU), validado cuantitativamente mediante una **sombra digital** basada en un modelo dinámico FOPDT (*First Order Plus Dead Time*).

---

## Estructura del Proyecto

```
.
├── config/                  # Archivos de configuración JSON (ganancias, reglas, parámetros FOPDT)
│   ├── dynamics_params.json
│   ├── flight_config.json
│   └── fuzzy_rules.json
├── core/                    # Módulos del núcleo de control y física
│   ├── digital_shadow.py
│   ├── dynamics_model.py
│   ├── fuzzy_tuner.py
│   ├── pid_controller.py
│   └── tello_interface.py
├── data/                    # Persistencia incremental y métricas de desempeño
│   ├── data_logger.py
│   └── metrics.py
├── docs/                    # Documentación por etapas
│   └── ARCHITECTURE.md
├── experiments/             # Orquestación de campañas experimentales por bloques
│   └── experiment_runner.py
├── gui/                     # Interfaz gráfica de usuario
│   └── gui_app.py
├── identification/          # Identificación de parámetros FOPDT (K, L, tau)
│   └── param_identification.py
├── missions/                # Lazo de control para perfil A -> B con seguridad RNF8
│   └── mission_a_to_b.py
├── tests/                   # Suite de pruebas unitarias offline con pytest
│   ├── test_dynamics.py
│   ├── test_fuzzy.py
│   ├── test_metrics.py
│   └── test_pid.py
├── Analisis.md              # Especificación técnica base
└── requirements.txt         # Dependencias fijadas del proyecto
```

---

## Descripción de Etapas e Implementación

### Etapa 1: Interfaz de Hardware y Abstracción Simulada (`core/tello_interface.py`)
- Define la clase `TelloInterface` que actúa como envoltorio sobre `djitellopy`.
- Proporciona un modo `use_mock=True` para desarrollo y pruebas offline completas sin necesitar el dron físico ni red Wi-Fi activa.

### Etapa 2: Controladores PID y FIS Mamdani (`core/pid_controller.py`, `core/fuzzy_tuner.py`)
- **PID Controller:** Implementa anti-windup clamping para evitar la saturación integral durante errores sostenidos sobre comandos RC `[-100, 100]`.
- **Fuzzy Tuner:** Utiliza `scikit-fuzzy` para cargar universos de discurso, funciones de membresía triangulares y matrices de reglas desde `config/fuzzy_rules.json`. Evalúa el error y la derivada para ajustar $\Delta K_p, \Delta K_i, \Delta K_d$ en tiempo real.

### Etapa 3: Sombra Digital FOPDT (`core/dynamics_model.py`, `core/digital_shadow.py`)
- Modelo matemático continuo $V(s)/U(s) = K e^{-Ls} / (\tau s + 1)$.
- Discretización exacta por Retención de Orden Cero (ZOH):
  $$v[k] = a \cdot v[k-1] + K(1-a) \cdot u[k - N_d]$$
- Utiliza un buffer circular para modelar el tiempo muerto $N_d = \text{round}(L / T_s)$.
- Calcula el residuo instantáneo $v_{\text{real}} - v_{\text{sombra}}$ como indicador de perturbación.

### Etapa 4: Persistencia Incremental y Métricas (`data/data_logger.py`, `data/metrics.py`)
- Escribe cada muestra inmediatamente a formato CSV con `flush()` explícito para tolerancia a caídas de conexión.
- Métricas calculadas: RMSE, MAE, ISE, IAE, ITAE, Settling Time (±5%), Rise Time (10%-90%), Varianza de Comando, Fidelidad de Sombra.

### Etapa 5: Misión de Vuelo A -> B y Seguridad RNF8 (`missions/mission_a_to_b.py`)
- Control continuo en lazo cerrado para velocidad $v_{gx}$.
- **Seguridad RNF8 integrada en Core:**
  1. Aterrizaje inmediato si la batería $< 15\%$.
  2. Aterrizaje inmediato si no se recibe telemetría por $> 2.0\text{ s}$.
  3. Soporte para solicitud de parada de emergencia manual desde la GUI o hilo externo.
  4. Bloqueo preventivo de despegue si viento natural $> 5.0\text{ m/s}$.

### Etapa 6: Interfaz Gráfica (`gui/gui_app.py`)
- Construida sobre Tkinter y Matplotlib.
- Totalmente desacoplada del Core (cumple la regla $GUI \to Core$).
- Incluye botón prominente de parada de emergencia siempre accesible.

---

## Ejecución de Pruebas Unitarias
Para ejecutar la suite de pruebas unitarias offline:

```bash
PYTHONPATH=. pytest
```
