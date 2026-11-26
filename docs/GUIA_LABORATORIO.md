# Guía de laboratorio (paso a paso)

Objetivo de la sesión: llegar con todo listo, **validar en orden** y terminar aterrizando
sobre el carro en movimiento. Cada paso dice qué correr, qué debe pasar y qué ajustar si no
pasa. Todos los ajustes se hacen en `config/settings.json`; no hay que tocar código.

Todos los comandos se corren desde la carpeta del repo:

```bash
cd ~/TMR_Landing
```

---

## 0. Antes de salir (5 min, en tu compu)

```bash
python3 -m pytest tests/unit -q
```
Deben pasar **51 pruebas**.

Opcional, pero muy recomendable: prueba la misión completa en
[Tello-Sim-Lab](https://github.com/DroneOps/Tello-Sim-Lab) (ver `docs/CAMBIOS.md`, sección 6).

```bash
PYTHONPATH=../Tello-Sim-Lab python3 -m tellosim.run --world tmr_landing --viewer --keys t@5 main.py
```

Debe terminar con `touched down ... (on it)`. Después borra o mueve los logs que dejó en `logs/`.

```bash
python3 tools/sim_mission.py --speed 20
```
Debe terminar con `ATERRIZÓ ... con error de X cm`, con X menor a 10.

**Material:**
- [ ] ArUco **DICT_6X6_100** (el de ustedes es el **ID 12**; con `marker_id: null` acepta cualquiera),
      impreso a **10.0 cm exactos** de lado negro. Mídelo con regla; si mide otra cosa, cambia
      `vision.marker_size_cm`. Pégalo plano en el centro del carro.
- [ ] **Papel MATE, sin enmicar ni plástico encima.** Un reflejo que cae sobre el borde negro
      del ArUco rompe su contorno y el detector no lo encuentra, aunque el resto se vea bien.
      Los reflejos dentro del marcador sí se toleran (corrige algunos bits); en el borde no.
- [ ] Baterías del Tello cargadas (el programa no despega con menos de 30 %).
- [ ] Carro cargado y emparejado por Bluetooth (ver `~/mobile_plataform/README.md`).
- [ ] Opcional: tablero de ajedrez de **9×6 esquinas interiores** para calibrar la cámara.

---

## 1. Sensores (2 min, sin volar)

```bash
python3 tests/test_telemetry.py
```

Debe pasar esto:
- La batería se lee.
- Al subir y bajar el dron con la mano, `tof` cambia.
- En reposo, `vel_x` y `vel_y` están en 0.

Sal con Ctrl+C.

---

## 2. Calibrar la cámara (10 min, opcional pero recomendado)

```bash
python3 tools/calibrate_camera.py --cols 9 --rows 6 --square-cm 2.5
```

1. Mueve el **tablero** (no el dron) frente al espejo, a distintas distancias e inclinaciones
   y en las esquinas de la imagen. Oprime `ESPACIO` cada vez que salga en verde (15–20 veces).
2. Oprime `c` para calcular y guardar. Un error RMS menor a 1.0 es bueno.

Esto mejora las distancias en cm del PnP. Si no hay tiempo, se puede volar con la matriz
aproximada, que es como funcionó hasta ahora.

---

## 3. Prueba en seco con el dron en la mano (5 min) ⚠️ no saltarse

```bash
python3 main.py --dry-run
```

1. Sostén el dron a ~1 m sobre el ArUco y oprime `t`. Debes ver los estados pasar por
   DESPEGUE → BÚSQUEDA → SEGUIMIENTO.
2. **Revisa los signos** viendo la línea `CMD` del HUD:

| Pones el ArUco físicamente... | Debe salir... |
|---|---|
| a la **derecha** del dron (visto desde atrás del dron) | `roll` **positivo** |
| a la **izquierda** | `roll` negativo |
| **adelante** del dron | `pitch` **positivo** |
| **atrás** | `pitch` negativo |

   Si un eje sale al revés, cambia el signo de `navigator.axis_sign_roll` o `axis_sign_pitch`.
   Con `flip_code=1` debería salir bien, porque es la configuración que ya funcionó en vuelo.
3. Centra el dron sobre el ArUco y bájalo despacio con la mano. Debe pasar a DESCENSO y,
   cerca del ArUco, a PUNTO_CIEGO.
4. Sal con `q`.

---

## 4. Vuelo 1: calibrar la velocidad (plataforma QUIETA, ~1 min de vuelo)

Este es el paso **más importante**. El simulador mostró que un signo de velocidad equivocado
hace fallar todo, aunque la plataforma esté quieta.

```bash
python3 main.py --solo-seguimiento
```

1. Carro **apagado y quieto**, con el dron encima. Oprime `t`.
2. Cuando esté en SEGUIMIENTO y estable, oprime `d`, espera 3 s, `a`, espera 3 s, `a`,
   espera 3 s, `d`. Luego haz lo mismo con `w` y `s`. Cada tecla mueve el objetivo 15 cm,
   así que el dron se desplaza y genera datos.
3. Oprime `q` (aterriza y sale).
4. Analiza el vuelo:

```bash
python3 tools/analyze_log.py
```

5. En la **sección 3** copia lo que sugiere a `navigator.vel_sign_x/y` y `vel_scale_x/y`,
   **solo si la correlación dice BUENA o REGULAR**. Si dice MALA, repite el vuelo moviéndolo más.
   Con el espejo del laboratorio y `flip_code=1` se espera **`vel_sign_x = -1`** (ya es el valor por
   defecto: lo midió Tello-Sim-Lab) y `vel_sign_y = +1`, con escala ~10. Si el vuelo real dice otra
   cosa, manda el vuelo real.
6. En la **sección 5**, `velocidad estimada de la plataforma` debe ser menor a 5 cm/s, porque
   la plataforma está quieta. Si es mayor, la escala o el signo siguen mal.

**Si el dron oscila** alrededor del ArUco: baja `kp` o sube `kd` en `navigator.pid_x/pid_y`.
**Si es lento** para centrarse: sube `kp` de a poco (por ejemplo +0.25).

Repite el vuelo con los valores nuevos y confirma que la sección 5 da menos de 5 cm/s.

---

## 5. Vuelo 2: seguir al carro en movimiento (sin bajar)

En otra terminal, pon el carro a ir y venir. Empieza **lento y con tramos largos**:

```bash
python3 ~/mobile_plataform/ida_y_vuelta.py 5
```

(5 s de ida y 5 s de vuelta. Baja `"base"` en `CONFIGURACION_CARRO` si va muy rápido.)

Luego, con el dron:

```bash
python3 main.py --solo-seguimiento
```

Debe pasar esto:
- El dron sigue al carro.
- Después de 2 cambios de sentido, el HUD muestra `Cambio de sentido cada X s` con un X
  parecido a 12 s (5 + 1 + 5 + 1).
- `Vplat` del HUD es parecido a la velocidad real del carro.

Después del vuelo:

```bash
python3 tools/analyze_log.py --plot
```

- La **sección 4** sugiere `feedforward_gain`. Usa ~80 % del valor sugerido.
- Si el dron **se queda atrás** del carro, sube `feedforward_gain`. Si **se adelanta** o
  sobrecorrige en los cambios de sentido, bájalo.

---

## 6. Vuelo 3: misión completa con la plataforma QUIETA

```bash
python3 main.py
```

`t` y no tocar. Debe hacer: SEGUIMIENTO → DESCENSO → PUNTO_CIEGO → **corte de motores a ~20 cm**.

| Si pasa esto | Ajusta |
|---|---|
| Nunca pasa a DESCENSO | Sube `fsm.center_radius_cm` (12 → 15) |
| Baja y regresa a SEGUIMIENTO muchas veces | Sube `fsm.descent_abort_radius_cm` o mejora el PID |
| Se queda flotando bajo sin aterrizar | El Tello no baja más: sube `fsm.touchdown_tof_cm` (20 → 25) |
| El golpe al caer es muy fuerte | Baja `fsm.touchdown_tof_cm`, o usa `"final_mode": "land"` |
| Cae siempre adelantado o atrasado respecto al ArUco | Ajusta `navigator.setpoint_y_cm` (el espejo está delante del centro del dron); igual con X |
| Pierde el ArUco muy alto en el descenso | Baja `mission.descent_rc_fast` |
| Se va hundiendo mientras sigue | Sube `mission.alt_hold_kp` o `sink_comp_rc` |
| Al buscar, se aleja de donde estaba la plataforma | Revisa el signo y la escala de la velocidad (paso 4): el control de posición sin ArUco los usa |

---

## 7. Vuelo 4: misión completa con el carro en movimiento

1. Carro con `ida_y_vuelta.py 5` (lento).
2. `python3 main.py` y luego `t`.
3. Es **normal** que a ~58 cm se quede siguiendo un rato con el aviso
   `APRENDIENDO RITMO DE LA PLATAFORMA` o `ESPERANDO CAMBIO DE SENTIDO`: está esperando una
   ventana segura entre dos cambios de sentido.
4. Cuando funcione, sube poco a poco la velocidad del carro y acorta los tramos (`ida_y_vuelta.py 3`).

| Si pasa esto | Ajusta |
|---|---|
| Espera mucho y nunca encuentra ventana | Usa tramos más largos (≥ 5 s), o baja `mission.final_phase_s` si en los logs la bajada final (DESCENSO debajo de 58 cm + PUNTO_CIEGO) dura menos de 5 s |
| Cae fuera justo cuando el carro cambia de sentido | Sube `mission.final_phase_s` |
| La bajada final tarda mucho | Sube `descent_rc_slow` y `blind_descent_rc`. Ojo: el Tello baja muy lento por rc (~0.15 cm/s por unidad) |
| `Cambio de sentido cada` da valores raros | Sube `navigator.reversal.min_speed_cm_s` o `confirm_s` |

---

## Si el ArUco refleja la luz (tarjeta enmicada)

Lo primero es imprimirlo en papel **mate**. Mientras tanto, `vision.corner_tracking` (activado por
defecto) sigue las esquinas por flujo óptico durante los cuadros en que el reflejo impide
detectarlo:

- **En el HUD:** el marcador se dibuja en **amarillo** y aparece `ARUCO SEGUIDO (flujo optico)`.
- **En el analizador:** la sección 5 dice qué porcentaje fue detectado y qué porcentaje seguido.
- **Si el cuadro amarillo se separa del ArUco real**, baja `max_track_s` (2 → 1) o apágalo
  con `"enabled": false`.

## Seguridad durante todas las pruebas

- **`ESPACIO` = corta motores al instante.** Ten el dedo listo.
- `l` = aterrizar normal · `b` = abortar descenso · `q` = aterrizar y salir.
- La ventana del video debe estar **seleccionada** para que funcionen las teclas.
- Si el programa truena o se oprime Ctrl+C, el dron aterriza solo (`try/finally`).
- Antes de cada vuelo revisa en el HUD que la batería esté arriba de 30 %.

## Después de cada vuelo

- Cada vuelo deja `logs/AAAAMMDD_HHMMSS.csv` y `.mp4`.
- `python3 tools/analyze_log.py` analiza el último. `--plot` muestra gráficas.
- **Anota** qué parámetros cambiaste y qué resultado dio cada vuelo. Sirve para explicarlo
  después y para no repetir pruebas.
