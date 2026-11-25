# Cambios realizados y cómo funciona el sistema

Este documento explica **qué se cambió, por qué, y cómo funciona cada pieza**, para poder
entenderlo y explicárselo al equipo. La guía práctica para volar está en
[GUIA_LABORATORIO.md](GUIA_LABORATORIO.md).

> Estado: cambios en la rama `feat/mision-completa`, pendientes de revisión antes de integrarse a `main`.

---

## 1. Resumen en una página

Antes había piezas sueltas que funcionaban por separado: visión, PID, Kalman, FSM y HUD.
Ya se había probado el seguimiento en X/Y en vuelo, pero `main.py` no despegaba, no usaba
la máquina de estados ni bajaba. Además, el Kalman no sabía que la plataforma se mueve.

Ahora:

| Qué | Antes | Ahora |
|---|---|---|
| `main.py` | Bucle de prueba sin despegue ni descenso | Misión completa guiada por la FSM, con control de altura, seguridad y log |
| Configuración | Números escritos en cada archivo | Todo en `config/settings.json` (con comentarios) |
| Kalman | Solo posición relativa | Posición **y velocidad de la plataforma**, más compensación de la latencia del video |
| Control X/Y | Solo PID | PID + **feedforward** con la velocidad de la plataforma |
| Cambios de sentido del carro | Sin manejar | `ReversalPredictor` aprende el ritmo y evita bajar justo antes de un cambio |
| Toque final | `land()` lento del Tello | Corte de motores (`emergency`) a 20 cm, configurable |
| Seguridad | Ninguna | Batería mínima, `try/finally` que siempre aterriza, tecla de emergencia |
| Registro | Ninguno | CSV + video por vuelo en `logs/` y analizador automático |
| Pruebas | Scripts manuales con el dron | 45 pruebas automáticas (`pytest`) + simulador propio + pruebas en **Tello-Sim-Lab** (sección 6) |

---

## 2. Arquitectura: cómo fluye un ciclo

`main.py` corre un ciclo a ritmo fijo (30 Hz). En cada vuelta:

```
 Tello ──frame──► VisionSystem ──pos_cm (X,Y,Z) o None──┐
   │                                                     ▼
   └──telemetría (vel, ToF, batería)──────────────► Navigator.update()
                                                     (Kalman X y Kalman Y)
                                                         │ posición estimada,
                                                         │ velocidad de la plataforma
                                                         ▼
                                              StateMachine.update()
                                                (decide el ESTADO)
                                                         │
                                                         ▼
                                    compute_commands(estado, ...)   ← en main.py
                                   (roll, pitch con PID+feedforward;
                                    throttle según el estado)
                                                         │
                                                         ▼
                                         TelloDriver.set_velocity()
                                         HUD + FlightLogger (CSV/video)
```

La separación de responsabilidades es:
- **Visión:** solo mide dónde está el ArUco. No decide nada.
- **Navigator:** estima (Kalman) y calcula correcciones horizontales (PID). No sabe de estados.
- **StateMachine:** solo decide el estado. **No manda comandos.**
- **main.py (`compute_commands`):** traduce el estado a comandos. Es el único lugar donde
  se decide la altura.

---

## 3. Cambios archivo por archivo

### 3.1 `config/settings.json` (antes vacío) y `config/loader.py` (nuevo)

- Todos los parámetros viven aquí, en secciones: `vision`, `navigator`, `fsm`, `mission`, `logging`.
- Las llaves que empiezan con `_` son **comentarios**. `loader.py` las quita al cargar, porque
  JSON no admite comentarios de verdad.
- Cada sección se pasa directo al constructor de su clase: `Navigator(**settings["navigator"])`.
  Si alguien escribe mal una llave, Python truena **al arrancar**, antes de despegar. La prueba
  `tests/unit/test_config.py` revisa esto sin dron.
- `save_camera_calibration()` lo usa `tools/calibrate_camera.py` para escribir la calibración.

### 3.2 `vision/aruco_tracker.py`

- Recibe del config: diccionario, ID, tamaño, matriz de cámara y distorsión.
- **Selección de marcador:** si hay varios, usa el ID configurado o, si no hay uno, el más
  grande en imagen (el más cercano).
- **Tamaño por ID** (`marker_sizes_cm`): permite poner un ArUco grande y otro chico en la
  plataforma y que cada uno dé la distancia correcta. Sirve para reducir el punto ciego más adelante.
- **Error corregido:** en OpenCV 5 los `ids` tienen otra forma; ahora se usa `ids.flatten()`.
  La prueba de humo lo detectó, y en el laboratorio habría tronado.

### 3.3 `vision/vision_system.py`

Solo pasa los parámetros nuevos al tracker.

> **Hallazgo importante sobre `flip_code`:** con `flip_code=0` (vertical) el ArUco
> *también* se detecta, porque queda girado 180° y eso sigue siendo un ArUco válido, pero
> **los signos de X e Y salen invertidos**. Antes `main.py`, `test_fsm.py` y `test_vision.py`
> usaban 0, y `test_tracking.py` (el que funcionó en vuelo) usaba 1. Ahora todos leen
> `flip_code=1` del config. Si alguien cambia `flip_code`, también tiene que cambiar los
> `axis_sign_*`. La prueba `test_vision_system_undoes_mirror` documenta esto.

### 3.4 `control/pid_controller.py`

Se conservó la misma interfaz y se agregaron mejoras:
- **`reset()`:** borra la integral y la derivada. Se llama al empezar a usar el PID (por
  ejemplo, al pasar de BÚSQUEDA a SEGUIMIENTO), para no arrastrar historia vieja.
- **Derivada sobre la medición** en vez de sobre el error. Con setpoint fijo es igual, pero
  evita el "golpe" (*derivative kick*) cuando reaparece el ArUco o cambia el setpoint. En la
  primera llamada después de `reset()` la derivada es 0.
- **Anti-windup:** la integral no crece mientras la salida está saturada en la misma
  dirección, y tiene un límite (`integral_limit`). Hoy `ki=0`, pero queda listo si se usa.
- **`dt` explícito** (opcional) y limitado a 0.2 s, para que un ciclo trabado no genere saltos.
- **`feedforward`:** término que se suma a la salida antes del límite (ver 3.6).
- **`clock` inyectable:** las pruebas usan un reloj falso para no tener que esperar.

### 3.5 `control/kalman.py`: el cambio más importante

Antes (`KalmanPosicion`) el estado era la posición relativa, y la "velocidad" solo era la
del dron copiada. Para una plataforma **móvil** eso no basta: el filtro no puede saber que
el ArUco se mueve solo, así que el PID siempre llega tarde.

Ahora (`KalmanObjetivo`), uno por eje:

```
Estado:   x = [ p , v_obj ]
          p     = posición del ArUco relativa a la cámara (cm)  ← la mide el PnP
          v_obj = velocidad de la plataforma (cm/s)             ← nadie la mide: se DEDUCE

Modelo:   p     ← p + dt · (v_obj − v_dron)
          v_obj ← v_obj          (velocidad constante; los cambios entran como ruido Q)
```

**Intuición:** si el dron está quieto (`v_dron = 0`) y aun así el ArUco se mueve en la imagen,
la diferencia la explica `v_obj`. Si el ArUco se mueve en la imagen *solo* porque el dron se
movió, `v_dron` lo explica y `v_obj` queda en 0. Por eso es tan importante que la velocidad
del dron tenga **la escala y el signo correctos** (sección 4).

Matrices, por si las piden:
- `F = [[1, dt], [0, 1]]`, `B = [[-dt], [0]]` (entrada: `v_dron`), `H = [1, 0]`.
- `Q` = modelo de aceleración aleatoria de la plataforma (`std_acc_target`) más un ruido extra
  de posición (`std_pos_process`). `R = std_vision²`.
- Además: `reset()`, `decay_target_velocity()` (olvida poco a poco `v_obj` cuando no se ve
  el ArUco) y `clamp_target_velocity()` (límite físico).

### 3.6 `logic/navigator.py`

Junta los dos Kalman y los dos PID.
- **Conversión de velocidad:** `v_dron_cm_s = vel_sign · vel_scale · velocidad_cruda_del_Tello`.
  El Tello normalmente reporta **dm/s**, por eso `vel_scale = 10`. Hay que **confirmarlo** con
  el analizador.
- **Compensación de latencia del video** (`vision_latency_s`): la imagen llega unos 100–200 ms
  tarde, así que muestra dónde *estaba* el ArUco. Antes de corregir el Kalman, la medición se
  adelanta: `medición + (v_obj − v_dron) · latencia`.
- **Reinicio automático:** si el ArUco estuvo perdido más de `reset_after_lost_s`, al
  reaparecer se reinician el filtro y los PID con la medición nueva.
- **Feedforward:** `comando = PID(posición estimada) − feedforward_gain · v_obj`. Así el dron
  empieza a moverse junto con la plataforma **sin esperar a acumular error**. En simulación, a
  20 cm/s, bajó el error de seguimiento de ~10 cm a ~7 cm.
- **Signos finales** (`axis_sign_roll = +1`, `axis_sign_pitch = −1`): es la convención que
  ya funcionaba en vuelo con `test_tracking.py`. No se cambió.
- `calculate_commands()` se conservó para que `tests/test_tracking.py` siga funcionando.
- Alimenta al `ReversalPredictor` solo con visión fresca y después de 1.5 s de seguimiento,
  porque al arrancar la velocidad estimada brinca.

### 3.7 `logic/reversal_predictor.py` (nuevo)

**Problema que resuelve:** los últimos ~25 cm del descenso son a ciegas, porque el ArUco ya
no se ve a través del espejo. Si el carro cambia de sentido justo ahí, el dron cae fuera. En
simulación esa era la causa principal de los aterrizajes fallidos.

**Cómo funciona:** el carro va y viene (`ida_y_vuelta.py`), con un ritmo regular. Este módulo:
1. Vigila la velocidad estimada de la plataforma. Cuando apunta al lado contrario que antes
   (producto punto negativo), registra un **cambio de sentido**. Solo cuenta velocidades de más
   de `min_speed_cm_s` sostenidas al menos `confirm_s`, para ignorar ruido.
2. El **periodo** es la mediana de los intervalos entre cambios.
3. `safe_window(needed, settle)` responde: "¿me da tiempo de hacer una maniobra de `needed`
   segundos antes del siguiente cambio, y ya pasaron `settle` segundos desde el último?".

Con la plataforma quieta nunca detecta movimiento y siempre responde "sí".

### 3.8 `logic/state_machine.py`

Los mismos estados y la misma lógica de antes, con estos cambios:
- Los tiempos y umbrales vienen del config (antes estaban fijos en el código).
- Recibe `error_xy` **estimado por el Kalman** (relativo al setpoint) en lugar de la X/Y cruda.
  Así la FSM y el PID usan el mismo dato. Si no se le pasa, funciona como antes.
- **Nuevo en DESCENSO:** si el error pasa de `descent_abort_radius_cm` durante
  `descent_abort_s`, regresa a SEGUIMIENTO en lugar de seguir bajando descentrado.
- **Nuevo en PUNTO_CIEGO:** si se acaba el tiempo pero ya está muy bajo
  (`blind_force_land_tof_cm`), aterriza en vez de abortar. El Tello a veces no baja más cerca
  del piso con rc. Si vuelve a ver el ArUco más arriba, regresa a DESCENSO.
- `clock` inyectable para las pruebas.

Diagrama de estados:

```
ESPERA ─[t]→ DESPEGUE ─(2 s)→ BUSQUEDA ─(ve ArUco 0.5 s)→ SEGUIMIENTO ─(centrado 1 s)→ DESCENSO
                                 ▲                              ▲  │                    │   │
                                 └──────(perdido 2 s)───────────┘  └─(descentrado 0.5 s)┘   │
                                 ▲                                                          │
                                 ├──(perdido estando alto)──────────────────────────────────┤
                                 │                                          (pierde ArUco bajo)
                                 └──(timeout alto)── PUNTO_CIEGO ◄──────────────────────────┘
                                                         │ (ToF < 20 cm, o timeout estando bajo)
                                                         ▼
                                                    ATERRIZAJE (corte de motores)
```

### 3.9 `main.py` (reescrito)

Qué hace en cada estado (`compute_commands`):

| Estado | X/Y (roll, pitch) | Altura (throttle) |
|---|---|---|
| ESPERA / DESPEGUE | 0 | 0 |
| DESPEGUE (estabilizando) | Mantiene la posición (frena la deriva) | 0 |
| BÚSQUEDA | Si se perdió hace menos de 1.5 s: sigue la **predicción** del Kalman. Si no: **mantiene la posición** integrando la velocidad del Tello; si en 12 s no aparece, recorre una **espiral cuadrada** | Sube con el **ToF** hasta `search_tof_cm` para ver más área. Como el carro va y viene en línea recta, suele volver a pasar por abajo |
| SEGUIMIENTO | PID + feedforward | **Mantiene altura con el ToF** (`track_tof_cm`): el Tello se hunde al moverse |
| DESCENSO | PID + feedforward | Baja **solo si está centrado** (`descent_gate_cm`): rápido arriba, lento debajo de 70 cm. Debajo de `commit_tof_cm` espera una **ventana segura** (ver abajo) |
| PUNTO_CIEGO | PID sobre la **predicción** (sin visión) | Baja a `blind_descent_rc` |
| ATERRIZAJE | — | Corta motores (`final_mode: "emergency"`) o `land()` |

**Compuerta de cambio de sentido (`reversal_gate`):** al llegar a `commit_tof_cm` (58 cm) el
dron se queda a esa altura siguiendo a la plataforma hasta que:
- ya conoce el ritmo de la plataforma (si se mueve), **y**
- el siguiente cambio de sentido está a más de `final_phase_s` (5 s), **y**
- pasaron al menos `reversal_settle_s` (0.8 s) desde el último cambio.

Después de eso se "compromete" y baja sin detenerse. Si la espera pasa de `max_gate_wait_s`
(20 s), baja de todos modos. El HUD muestra "APRENDIENDO RITMO..." o "ESPERANDO CAMBIO DE
SENTIDO" mientras espera.

**Otros detalles:**
- **Seguridad:** no despega con batería < 30 %; aterriza si baja de 15 % en vuelo;
  `try/finally` → **siempre** aterriza y cierra todo, aunque el programa truene o se oprima Ctrl+C.
- **Teclas:** `t` despegar · `ESPACIO` emergencia · `l` aterrizar · `b` abortar descenso ·
  `q` salir. Con `--solo-seguimiento` también funcionan `w`/`a`/`s`/`d`.
- **Modos:** `--dry-run` (nunca despega, para probar con el dron en la mano) y
  `--solo-seguimiento` (vuela, sigue y nunca baja; sirve para calibrar).
- **Errores viejos corregidos:** ruta de `sys.path` (apuntaba arriba del repo), `imshow` con
  frame `None`, el dron nunca despegaba, `dt` fijo de 0.1 en el Kalman.
- Ritmo fijo del ciclo (`loop_hz`), y el `dt` real se mide en cada vuelta.

### 3.10 `control/tello_driver.py`

- `emergency()`: corta los motores (toque final).
- `hover()` y `shutdown()` (cierre seguro: aterriza si sigue volando y luego `end()`).
- `set_velocity` limita los valores a −100..100 y ya no llena la consola de mensajes cuando
  los motores están apagados (`verbose=False`).
- `move_up` se limita al rango válido del Tello (20–500 cm).

### 3.11 `ui/telemetry_hud.py`

Parámetro opcional `extra_lines`: líneas abajo a la izquierda con el error estimado, la
velocidad de la plataforma, los comandos, los Hz del ciclo y los avisos.

### 3.12 Nuevos: registro y herramientas

| Archivo | Para qué |
|---|---|
| `utils/flight_logger.py` | Un CSV por vuelo (+ video con HUD) en `logs/AAAAMMDD_HHMMSS.*`. `logs/` está en `.gitignore` |
| `tools/analyze_log.py` | Lee el CSV y reporta Hz, estados, **escala y signo de la velocidad**, **feedforward sugerido**, error de seguimiento. `--plot` grafica |
| `tools/calibrate_camera.py` | Calibra la cámara a través del espejo con un tablero de ajedrez y escribe la calibración en `settings.json` |
| `tools/sim_mission.py` | Simulador de la misión completa con el **mismo código** de main.py, un carro que va y viene, latencia de video y velocidades enteras. `--speed`, `--leg`, `--plot` |

**Cómo el analizador saca la escala y el signo:** con la plataforma quieta, todo movimiento
del ArUco en la imagen se debe al dron, así que `dp/dt = −k · vel_tello`. Ajusta `k` por
mínimos cuadrados; `signo(k)` va en `vel_sign` y `|k|` en `vel_scale`. Para el feedforward,
compara el comando rc con la velocidad del dron (probando retrasos de 0 a 1 s) y saca cuántos
cm/s produce cada unidad de rc.

### 3.13 Pruebas

- `tests/unit/` (**nuevo**, corre con `python3 -m pytest tests/unit -q`, sin dron, en < 1 s):
  - `test_config.py`: el JSON carga y cada sección arma su clase.
  - `test_pid_unit.py`, `test_kalman_unit.py`, `test_reversal_unit.py`, `test_fsm_unit.py`.
  - `test_vision_unit.py`: ArUco sintéticos (distancia, signo, filtro de ID, espejo).
  - `test_mission_sim.py`: misiones completas simuladas (quieta y a 10–20 cm/s) deben
    aterrizar a menos de 5–10 cm del centro, y el feedforward debe mejorar el seguimiento.
- Scripts manuales con dron (`tests/test_*.py`): `test_tracking`, `test_fsm` y `test_vision`
  ahora leen `settings.json`. `test_telemetry.py` estaba vacío y ahora imprime los sensores.
  **`test_pid.py` ya estaba roto desde antes** (usa `tracker.detect()`, que ya no existe) y no se tocó.

---

## 4. Lo que el simulador dice (y lo que NO)

Resultados con los parámetros actuales (error de aterrizaje en cm, 4–6 corridas):

| Plataforma | Tramos 3 s | Tramos 5 s |
|---|---|---|
| Quieta | 0–1 | 0–1 |
| 10 cm/s | 2–4 | 2–4 |
| 20 cm/s | 3–6 | 4–6 |
| 30 cm/s | 3–9 | 6–7 |

Sensibilidad (plataforma a 20 cm/s):
- **Signo de velocidad invertido → 5–20 cm de error, incluso con la plataforma quieta.**
  Es el parámetro más crítico. Se verifica en el primer vuelo.
- Escala 10× equivocada (si el Tello reportara cm/s) → ~10 cm. Afecta, pero no es catastrófico.
- Dron más "flojo" de lo supuesto (rc_gain 0.6) → ~13 cm. Se corrige subiendo `feedforward_gain`.
- Más latencia de video o más ruido en la visión → poco efecto.

**Lo que NO modela:** viento o turbulencia cerca del carro, el rebote al cortar motores,
la dinámica real del Tello, que el ToF mida el piso en lugar del carro en los bordes, ni
problemas de iluminación. Por eso hay que volar.

---

## 5. Pendientes conocidos

1. **Verificar en vuelo** el signo y la escala de la velocidad, y el feedforward (guía, pasos 4–5).
2. **Setpoint con espejo:** si el dron siempre cae adelantado o atrasado respecto al ArUco,
   ajustar `setpoint_y_cm` (el espejo está delante del centro del dron).
3. **Punto ciego:** un segundo ArUco chico en el centro (`marker_sizes_cm`) haría más corta
   la parte a ciegas.
4. **Reglas TMR 2027:** confirmar tiempos, tamaño y velocidad de la plataforma, si se permite
   el espejo y cómo se califica.
5. El ToF mide la distancia al *carro* solo cuando el dron está encima. Si en los bordes mide
   el piso, puede confundir la FSM; hay que revisarlo en los logs.

---

## 6. Pruebas en el simulador Tello-Sim-Lab (noviembre 2026)

[Tello-Sim-Lab](https://github.com/DroneOps/Tello-Sim-Lab) reemplaza a `djitellopy` sin cambiar
el código. Su modelo del Tello está **ajustado con vuelos reales del laboratorio**: zona muerta
del rc, retraso, hundimiento al moverse, deriva, retraso del video, ToF y espejo. Trae el mapa
`tmr_landing`: un ArUco de 10 cm en una plataforma que va y viene 1 m a cada lado a 10 cm/s.

### Cómo correrlo

No hace falta instalarlo: basta clonarlo junto al proyecto y apuntar `PYTHONPATH` a esa carpeta.

```bash
git clone https://github.com/DroneOps/Tello-Sim-Lab.git ../Tello-Sim-Lab
PYTHONPATH=../Tello-Sim-Lab python3 -m tellosim.run --world tmr_landing --keys t@5 --seconds 120 main.py
```

- `--headless` corre sin ventanas, `--viewer` agrega la vista desde arriba y
  `--record run.mp4` graba la corrida.
- Para probar condiciones difíciles: `--fog '{"visibility_cm": 300, "patches": 0.5}'` (humo) y
  `--model lab_tello7` (un Tello que deriva solo ~12 cm/s).
- Al final, el simulador imprime **dónde aterrizó de verdad**, por ejemplo
  `touched down 6 cm from its center (on it)`.
- ⚠️ `main.py` guarda sus logs en `logs/`. Para no mezclarlos con vuelos reales, usa una copia
  de `settings.json` con otro `logging.directory` y pásala con `--config`.

### Lo que encontró y cómo se corrigió

| # | Problema (cómo se vio) | Corrección |
|---|---|---|
| 1 | **Signo de la velocidad en X invertido.** El Kalman veía la plataforma a 43 cm/s cuando iba a 10, y el dron oscilaba ±20 cm con roll saturado | `vel_sign_x = −1`. Motivo: el espejo del laboratorio voltea arriba-abajo y `flip_code=1` voltea izquierda-derecha, así que la imagen queda girada 180° y la X de la cámara apunta a la **izquierda** del dron, al revés de `vgy` (derecha). El analizador lo confirmó con la plataforma quieta (correlación 0.96). **El código anterior tenía el mismo error** |
| 2 | **El dron se hundía mientras seguía.** Con throttle 0 el Tello baja 5–8 cm/s al moverse: pasó de 123 a 41 cm en 22 s y perdió el ArUco | Control de altura con el ToF (`hold_altitude`) en SEGUIMIENTO, en BÚSQUEDA y mientras espera ventana, más compensación anticipada del hundimiento (`sink_comp_rc`) cuando se mueve en roll y pitch a la vez |
| 3 | **Bajada mucho más lenta de lo supuesto.** El Tello baja ~0.15 cm/s por unidad de rc (medido): con rc −25 la "fase final de 1.6 s" tardaba ~10 s | `descent_rc_*` 60/50 y `blind_descent_rc` 60; `final_phase_s` 5 s |
| 4 | **Zona muerta del rc.** Debajo de rc ~5 el Tello no se mueve, así que las correcciones chicas no hacían nada | `rc_deadband: 5`: el Navigator suma la zona muerta a todo comando no nulo. `feedforward_gain` 0.7 (≈ 1/1.45 cm/s por unidad de rc, medido) |
| 5 | **La altura `h` del Tello no es confiable** (llegó a 0 en pleno vuelo) | BÚSQUEDA sube con el ToF (`search_tof_cm`) |
| 6 | **La altura de espera caía dentro de la zona de punto ciego.** Con humo, al perder el ArUco en un cambio de sentido a 47 cm, la FSM lo tomaba como punto ciego y bajaba a ciegas: cayó a 69 cm | `blind_tof_cm` 45 → 40 (medido: el ArUco se pierde a ToF ≈ 39), `commit_tof_cm` 50 → 58, y una prueba en `test_config.py` que impide que vuelvan a traslaparse |
| 7 | **Espera innecesaria para aprender el ritmo.** Con tramos de 20 s, juntar 2 cambios de sentido tardaba hasta 40 s | Después del primer cambio, `ReversalPredictor.min_leg` usa como cota inferior (conservadora) del tramo el tiempo que la plataforma llevaba moviéndose |
| 8 | **Con rc 0 el Tello no se queda quieto.** El Tello #7 derivó 10 m mientras "buscaba"; además `move_up` bloquea varios segundos sin control | BÚSQUEDA y DESPEGUE mantienen la posición integrando `vgx`/`vgy` (no dependen de la cámara). Espiral cuadrada si no aparece en 12 s. `takeoff_extra_cm: 0`: la subida la hace BÚSQUEDA con rc |
| 9 | El simulador propio (`tools/sim_mission.py`) suponía un dron ideal y por eso no detectó 2–4 | Ahora usa el modelo medido: zona muerta, subida y bajada asimétricas, hundimiento y retraso de 0.75 s |

### Resultados con la configuración final

| Escenario | Antes | Después |
|---|---|---|
| Plataforma quieta | 26 cm, **fuera** | 2–13 cm, sobre la plataforma (5/5) |
| Plataforma móvil (10 cm/s) | 27 cm, **fuera** | 5–12 cm, sobre la plataforma (6/6) |
| Plataforma móvil **con humo** (3 m de visibilidad) | — | 8–21 cm, sobre la plataforma (4/4) |
| Tello #1 del laboratorio | — | 5 cm (1/1) |
| Tello #7 (deriva ~12 cm/s) | se iba a 10 m | 2–22 cm (3/3) |

"Sobre la plataforma" lo dice el simulador (la plataforma mide 50 cm). El simulador corre en
tiempo real y no es determinista: la misma semilla puede dar resultados distintos.

### Limitaciones que siguen

- **Tramos cortos:** la bajada final tarda ~4–5 s. Si el carro cambia de sentido más seguido
  (por ejemplo `ida_y_vuelta.py 3`: un cambio cada 4 s), no hay ventana segura: después de
  `max_gate_wait_s` baja de todos modos y suele fallar. Usar tramos de ≥ 5 s.
- **Posición sin ArUco:** integrar `vgx`/`vgy` (dm/s enteros) acumula error con el tiempo. Con
  un Tello que deriva mucho y una búsqueda larga, puede perderse. Para una posición sin ArUco
  de verdad, Tello-Sim-Lab trae `tellolab/floor_flow.py` (flujo óptico del piso por el espejo).
- **El simulador no modela** reflejos (como el de la tarjeta enmicada), suciedad del espejo
  ni cortes de WiFi, y su velocidad horizontal es ~1.6× más rápida que la medida con el clip
  del espejo. Los parámetros siguen necesitando confirmarse en vuelo real con
  `tools/analyze_log.py`.
- Si el espejo voltea la imagen de lado (`--camera-set mirror_axis=h`), los signos cambian y el
  dron se aleja (comprobado). Revisar los signos con `--dry-run` (paso 3 de la guía) si cambia el clip.
