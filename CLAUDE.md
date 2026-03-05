# CLAUDE.md — Segway Gemelo Digital
> Estado al 2026-03-04 | MATLAB/Simulink R2025b

---

## 1. QUE ES ESTE PROYECTO

Gemelo digital de un segway autobalanceado (pendulo invertido sobre dos ruedas).
Implementa un controlador LQR de dos lazos desacoplados y lo valida en Simscape Multibody 3D.

**Estados ODE:** `X = [theta, dtheta, x, dx, alpha, dalpha, int_ev, theta_rider_filt, alpha_ref_filt]`
- Estados 1-6: planta fisica
- Estados 7-9: controlador (integrador + filtros de referencia)

**Entradas:** `U = [V_R, V_L]` (voltaje motor derecha e izquierda)

**Convencion de signos:**
- `theta > 0` -> inclinado hacia adelante
- `alpha > 0` -> giro izquierda (RHR sobre eje vertical)

---

## 2. ARCHIVOS PRINCIPALES (raiz)

| Archivo | Rol | Estado |
|---------|-----|--------|
| `params.m` | **Correr primero siempre.** Define parametros fisicos y M11/M22/M33/det0 | estable |
| `ModelKane_final.m` | Derivacion simbolica Kane/Lagrange -> genera `segway_modelo_resultados.mat` | estable |
| `LQR_Con_SegVelocidad.m` | **Script principal activo (v5).** Disena K1_aug y K2, corre ODE 9 estados, llama SimscapeConstruct, grafica | estable |
| `SimscapeConstruct.m` | Construye modelo Simscape v4.1 (Planar Joint) desde cero y simula. Deja `out` en workspace | estable |
| `run_comparacion_LQR.m` | Comparacion ODE Lineal vs No Lineal (figura tesis) | estable |
| `segway_modelo_resultados.mat` | Cache del modelo simbolico (evita re-derivar cada vez) | no tocar |
| `simscape_context.txt` | Lista completa de bloques disponibles en las librerias Simscape de 2025b | referencia |
| `grid_floor.stl` | STL de cuadricula visual 40x40cm, generado automaticamente | se sobreescribe |

**Archivos obsoletos / versiones anteriores (NO usar como referencia activa):**

| Archivo | Nota |
|---------|------|
| `build_v4.m` | Constructor de v4 — precursor de SimscapeConstruct.m |
| `run_v4.m` | Runner de v4 — no reconstruye el modelo |
| `Simscape_LQR_v5_lts.m` | Version anterior del script principal, sustituida por LQR_Con_SegVelocidad.m |
| `Segway_Testbench_v4.slx` | Modelo Simscape de v4, construido con build_v4.m |
| `Segway_Testbench_v5.slx` | Generado automaticamente por SimscapeConstruct.m — se sobreescribe en cada run |
| `kane_final.m` | Version anterior de ModelKane_final.m |

**Carpeta `Debug/`:** `debug_estados.png`, `debug_trayectoria.png` y `debug_log.txt` — generados automaticamente al final de cada run.

---

## 3. FLUJO DE EJECUCION

```
params.m  ->  ModelKane_final.m (solo si no existe .mat)
          ->  LQR_Con_SegVelocidad.m
                |-- Disena K1_aug (4 estados + integrador)
                |-- Disena K2 (yaw)
                |-- Corre ODE ode45 (9 estados, referencia no lineal)
                '-- SimscapeConstruct.m
                        |-- Genera grid_floor.stl (cuadricula visual)
                        |-- Construye modelo Simscape v4.1 desde cero
                        |-- Inserta LQR_Controller + ForceDecomp como MATLAB Functions
                        '-- Simula -> out
          ->  Diagnostico + graficas + Debug/
```

---

## 4. ARQUITECTURA DE CONTROL (v5 actual)

### Desacoplamiento natural
Las ecuaciones de Kane desacoplan avance y giro exactamente:
- **Suma** de torques -> gobierna `[theta, dtheta, x, dx]`
- **Diferencia** de torques -> gobierna `[alpha, dalpha]`

Entradas virtuales: `Va = (VR+VL)/2`, `Vd = (VR-VL)/2`

### K1_aug — Avance con Lean-to-Speed
Estados: `[theta - theta_rider, dtheta, dx, int_ev]`
- `theta_rider` = referencia de inclinacion del rider (filtrada con `tau_rider`)
- `v_ref = Kv * theta_rider` (lean-to-speed)
- `int_ev = integral(dx - v_ref) dt` con anti-windup `+-int_max`
- Disenado con `Q1a = diag([2000, 100, 50, 50/(1.5*5)])`, `R1 = 1`
- Polos resultantes: -0.401, -0.820, -4.475, -17.709

### K2 — Yaw
Estados: `[alpha - alpha_ref, dalpha]`
- `alpha_ref` llega filtrada con `tau_alpha = 1.2 s` (filtro 1er orden)
- Disenado con `Q2 = diag([1800, 120])`, `R2 = 1`
- Polos resultantes: -4.066, -4.066

### Ganancias actuales
```matlab
K1a = [-120.506  -28.525  -10.306  -2.582]
K2  = [42.426  17.350]
```

### Saturaciones (tres niveles)
1. `+-V_sat_a = 24V` sobre Va
2. `+-V_sat_d = 24V` sobre Vd
3. `+-V_sat_f = 24V` sobre VR y VL finales

### Parametros del escenario (LQR_Con_SegVelocidad.m)
```matlab
Kv        = 5.0      % ganancia lean-to-speed [m/s/rad]
th_lean   = 5 deg    % inclinacion del rider
t_lean    = 3.0 s    % rider se inclina
t_back    = 13.0 s   % rider se incorpora
tau_rider = 0.3 s    % filtro rider
v_max     = 2.0 m/s  % velocidad maxima
int_max   = 4.0 m    % limite anti-windup
alpha_ref = 30 deg   % referencia de giro
t_avance_a= 8.0 s    % inicio giro
t_stop_a  = 12.0 s   % fin giro
tau_alpha = 1.2 s    % filtro alpha_ref
t_sim     = 20 s     % tiempo total de simulacion
```

---

## 5. ARQUITECTURA SIMSCAPE (SimscapeConstruct.m v4.1)

### Cadena cinematica
```
World -> RT_AxleHeight(+Z, r)
      -> Planar Joint (Px=world X, Py=world Y, Rz=yaw)
      -> RT_ThetaAxis(+X, -90 deg)
      -> Joint_theta(pitch, ComputedMotion)
      -> RT_BodyCM(-Y, l)
      -> Body_Solid
```
Ruedas bifurcan desde `Joint_planar/RConn1` -> `RT_WheelR/L` -> `Joint_phi_R/L`.

**Planar Joint port mapping (con todo sensing + actuation activo):**
- LConn: 1=base_frame, 2=Fx(Px), 3=Fy(Py), 4=Tz(Rz)
- RConn: 1=follower, 2=px, 3=vx, 4=py, 5=vy, 6=alpha, 7=dalpha

**Por que Planar Joint (v4) y no Joint_alpha + Joint_fwd (v3):**
En v3, `Joint_alpha` estaba en el origen del mundo. Al avanzar x metros, la inercia yaw crecia como `M*x^2` (teorema de Steiner/ejes paralelos). A x=4.4m el M33 efectivo era ~2084 vs 12.8 esperado (factor 163x). El Planar Joint combina traslacion + rotacion en el mismo punto, eliminando Steiner.

### Fuerzas aplicadas
| Fuerza | Gain | Destino |
|--------|------|---------|
| Torque rueda derecha | `alm * VR` | `Joint_phi_R` |
| Torque rueda izquierda | `alm * VL` | `Joint_phi_L` |
| Fuerza avance Fx | `(2*alm/r)*Va*cos(alpha)` | `Planar Joint Px` |
| Fuerza avance Fy | `(2*alm/r)*Va*sin(alpha)` | `Planar Joint Py` |
| Fuerza lateral (restriccion) | `-k_lat * v_lat` descompuesta en Fx,Fy | `Planar Joint Px,Py` |
| Torque yaw | `(d*alm/r) * Vd` | `Planar Joint Rz` |

### ForceDecomp (MATLAB Function)
Combina la fuerza de avance con la restriccion lateral:
```matlab
function [Fx, Fy] = ForceDecomp(F_fwd, vx, vy, alpha)
    ca = cos(alpha); sa = sin(alpha);
    v_lat = -vx*sa + vy*ca;       % velocidad perpendicular al heading
    k_lat = 10000;                 % friccion virtual de neumatico [N/(m/s)]
    F_lat = -k_lat * v_lat;       % fuerza que elimina deslizamiento lateral
    Fx = F_fwd*ca + F_lat*(-sa);  % componente mundo X
    Fy = F_fwd*sa + F_lat*ca;     % componente mundo Y
end
```

### Proyeccion de velocidad
El Planar Joint reporta vx, vy en frame mundo. Para obtener la velocidad de avance:
```matlab
dx_fwd = vx*cos(alpha) + vy*sin(alpha)
```
Y la posicion de avance: `x_fwd = integral(dx_fwd) dt`

### Sensing (PS2SL)
Los joints en 2025b exponen senales fisicas como `RConn`, NO como `Outport`.
- `RConn(1)` = conexion mecanica (frame)
- `RConn(2)` = posicion (con `SensePosition='on'`)
- `RConn(3)` = velocidad (con `SenseVelocity='on'`)
- **SIEMPRE** hacer `set_param(mdl,'SimulationCommand','update')` antes de leer `pH.RConn`

### Logging (To Workspace)
`th_log, dth_log, x_log, dx_log, al_log, dal_log, intev_log, VRVL_log, VaVd_log, px_log, py_log`
Acceso: `out.get('nombre_log')` o `out.tout`.

### Elementos visuales (sin efecto en fisica)
- **Suelo:** Brick Solid 30x8x0.05m, gris claro `[0.90 0.90 0.90]`, masa 1e6 kg (rigido a World)
- **Cuadricula:** File Solid desde `grid_floor.stl`, gris medio `[0.5 0.5 0.5]`, masa 0.001 kg, cuadricula 40x40cm
- **File Solid** usa `ExtGeomFileName` (no `FileName`), requiere `'UnitType','Custom','ExtGeomFileUnits','m'`

---

## 6. PARCHES / SIMPLIFICACIONES DEL MODELO SIMSCAPE

Tres simplificaciones reemplazan una sola cosa: **el contacto rueda-suelo con rolling constraint**.

### Parche 1: Fuerza de avance directa (Gain_F)
`F = (2*alm/r) * Va` inyectada directo al Planar Joint.
**Realidad:** el torque motor gira la rueda y la friccion rueda-suelo genera la traccion.
**Por que:** Rack & Pinion, Transform Sensor y senales basadas en phi fallaron en 2025b programaticamente. Spatial Contact Force dispara el coste computacional.

### Parche 2: Torque yaw directo (Gain_tau_al)
`T = (d*alm/r) * Vd` inyectado directo al eje Rz del Planar Joint.
**Realidad:** el yaw deberia venir de la diferencia de traccion entre ruedas transmitida via suelo.
**Mismo motivo:** sin rolling constraint no hay forma de transmitir la diferencia de torques al suelo.

### Parche 3: Restriccion lateral virtual (ForceDecomp, k_lat=10000)
`F_lat = -k_lat * v_lat` donde `v_lat = -vx*sin(alpha) + vy*cos(alpha)`.
**Realidad:** los neumaticos impiden el deslizamiento lateral.
**Por que:** el Planar Joint permite movimiento en cualquier direccion. Sin esta fuerza, el robot deriva lateralmente en las curvas por inercia. La junta prismatica de v3 resolvia esto pero causaba el bug de Steiner.

### Consecuencia de los parches
Las ruedas (`Joint_phi_R/L`) reciben torque y giran, pero no transmiten fuerza al suelo. Son inerciales (su `Iw` si contribuye) pero su rotacion `phi` no esta vinculada a la traslacion `Px`. En Mechanics Explorer la velocidad angular de las ruedas puede no coincidir exactamente con `dx/r`.

### Que si es fisica real
- Masas, inercias, geometria (params.m)
- Gravedad -9.81 m/s^2
- Dinamica de pitch (theta) — Simscape la resuelve completa
- Torque motor a ruedas: tau = alm * V

---

## 7. PARAMETROS FISICOS (params.m)

```matlab
M   = 80 kg       % masa cuerpo + persona
m   = 2 kg        % masa por rueda
r   = 0.20 m      % radio de rueda
d   = 0.60 m      % separacion entre ruedas
l   = 0.90 m      % distancia eje -> CM cuerpo
g   = 9.81 m/s^2
Icy = 10 kg*m^2   % inercia cuerpo pitch
Icz = 12 kg*m^2   % inercia cuerpo yaw
Icx = 12 kg*m^2   % inercia cuerpo roll
Iw  = 0.08 kg*m^2 % inercia rueda (spin)
Iwz = 0.04 kg*m^2 % inercia rueda (transversal)
alm = 2.0 N*m/V   % ganancia par motor
bem = 1.5 N*m*s/rad % back-EMF (NO usado — ver bugs #9)
```

**M33 (inercia yaw efectiva):**
```
M33 = Icz + 2*m*(d/2)^2 + 2*Iw*(d/(2*r))^2 + 2*Iwz = 12.80 kg*m^2
```

---

## 8. DECISIONES DE DISENO CRITICAS

### Sin back-EMF
El torque usa `tau = alm*V` (sin `bem`). La ODE y Simscape describen la misma planta -> K estabiliza Simscape. Incluir `bem` solo en la ODE hace que K vea una planta diferente a Simscape y el sistema diverge.

### Sin rolling constraint
Intentados: Rack & Pinion, senales basadas en phi, Transform Sensor — todos fallaron en 2025b. Solucion: inyectar fuerzas y torques directos (parches 1-3).

### PositionTargetSpecify
- `Joint_theta`: `PositionTargetSpecify='on'` con `PositionTargetPriority='Low'` — solo para condicion inicial.
- Planar Joint: sin PositionTargetSpecify en ningun eje.
- Siempre pasar `PositionTargetValue` en **grados**, nunca en radianes.

### Joint_theta sin torque actuado
`TorqueActuationMode='NoTorque'`, `MotionActuationMode='ComputedMotion'`. El pitch viene de la interaccion fuerzas + gravedad, no de un torque directo en el joint.

### Trayectoria XY reconstruida en postproceso
Aunque `px_log` y `py_log` existen (posicion world del Planar Joint), la trayectoria se reconstruye con cumtrapz para consistencia con la ODE:
```matlab
xw_sm = cumtrapz(t_sm, dx_sm .* cos(alpha_sm));
yw_sm = cumtrapz(t_sm, dx_sm .* sin(alpha_sm));
```

### ODE con 9 estados (no 6)
Estados 7-9 son del controlador, no de la planta:
- X(7) = `int_ev` = integral del error de velocidad (anti-windup)
- X(8) = `theta_rider_filtrado` = referencia de pitch filtrada con tau_rider
- X(9) = `alpha_ref_filtrado` = referencia de yaw filtrada con tau_alpha

Son necesarios para que la ODE filtre las referencias identicamente a los Transfer Fcn de Simscape. Sin el estado 9, la ODE veia escalon duro en alpha_ref mientras Simscape la filtraba -> mismatch.

---

## 9. BUGS CONOCIDOS Y CORREGIDOS

| # | Bug | Causa | Correccion |
|---|-----|-------|-----------|
| 1 | Yaw muy lento en Simscape | `PositionTargetSpecify='on'` con prioridad High en `Joint_alpha` — el joint resistia el torque del LQR | Eliminar `PositionTargetSpecify` de `Joint_alpha` |
| 2 | Pitch bloqueado en Simscape | `PositionTargetPriority='High'` en `Joint_theta` | Cambiar a `'Low'` |
| 3 | M33 incorrecto | `2*Iw*(d/(2*r)^2)` — precedencia de operadores | `2*Iw*(d/(2*r))^2` |
| 4 | M33 faltaba factor 2 en Iwz | `+ Iwz` en vez de `+ 2*Iwz` | Anadir factor 2 |
| 5 | PositionTargetValue en radianes | Pasaba 0.0873 rad como target | Pasar en grados (5 deg) |
| 6 | SimscapeLogType como set_param | `set_param(mdl,'SimscapeLogType',...)` no existe | `Simulink.SimulationInput.setModelParameter` |
| 7 | pH.RConn vacio o incompleto | Leer port handles sin update previo | `set_param(mdl,'SimulationCommand','update')` antes de `get_param` |
| 8 | alpha_ref como escalon duro | ODE sobreoscilaba, Simscape filtraba con Transfer Fcn -> mismatch | Filtro 1er orden `tau_alpha=1.2s` en ambos |
| 9 | ODE no filtraba alpha_ref | ODE usaba escalon duro para alpha_ref, Simscape lo filtraba con tau_alpha | Anadir estado 9 a la ODE: `dX9 = (al_target - X(9))/tau_alpha` |
| 10 | debug_log no se actualizaba | `dy_sm = out.get('dy_log')` fallaba silenciosamente en try/catch, abortando todo el bloque de debug | Eliminar esa linea, calcular yw_sm con cumtrapz |
| 11 | Inercia yaw crecia con la distancia (Steiner) | v3: `Joint_alpha` en el origen del mundo. Al avanzar x metros, M33_eff = M33 + M*x^2. A x=4.4m -> 2084 vs 12.8 (163x) | **Reescritura completa a v4:** Planar Joint (Px,Py,Rz) — yaw gira alrededor de la posicion actual |
| 12 | Robot derivaba lateralmente en curvas (3D) | Planar Joint permite movimiento libre en XY. Inercia del robot lo llevaba en linea recta al girar | ForceDecomp con restriccion lateral virtual `k_lat=10000 N/(m/s)` |
| 13 | File Solid: "unit could not be read" | Parametro de unidades requiere activacion explicita | `'UnitType','Custom','ExtGeomFileUnits','m'` |
| 14 | File Solid: "no parameter named FileName" | Nombre incorrecto del parametro | Es `'ExtGeomFileName'`, no `'FileName'` |

---

## 10. ESTADO ACTUAL — ULTIMO RUN EXITOSO

**Run 2026-03-04 15:37:21** — Todos los sistemas funcionando:

```
Evento       th_ODE    th_SM   dx_ODE    dx_SM   al_ODE    al_SM   yw_ODE    yw_SM     slip
--------------------------------------------------------------------------------------------
t=0           0.000    0.000    0.000    0.000     0.00     0.00    0.000    0.000    0.000
t_lean       -0.000    0.000    0.000    0.000     0.00     0.00    0.000    0.000    0.000
t_giro       -0.707   -0.495    0.759    0.698    -0.00    -0.00   -0.000    0.000    0.000
t_recto      -0.218   -0.137    0.513    0.508    28.46    28.45    0.720    0.695    0.000
t_back       -0.150   -0.098    0.488    0.488    18.01    17.93    0.928    0.902    0.000
t_fin         0.426    0.252   -0.160   -0.132     0.05     0.05    0.968    0.923    0.000
```

**Metricas clave:**
- Alpha match: 28.45 vs 28.46 deg en t_recto (error < 0.04%)
- Theta final: 0.252 deg (converge a 0)
- Alpha final: 0.053 deg (converge a 0)
- Trayectoria: yw_SM=0.923 vs yw_ODE=0.968 (4.5 cm de diferencia)

---

## 11. API SIMSCAPE 2025b — REFERENCIA RAPIDA

```matlab
% Parametros VALIDOS en set_param para juntas Revolute:
'TorqueActuationMode'    -> 'InputTorque' | 'NoTorque'
'MotionActuationMode'    -> 'ComputedMotion'
'SensePosition'          -> 'on' | 'off'
'SenseVelocity'          -> 'on' | 'off'
'DampingCoefficient'     -> '0'
'PositionTargetSpecify'  -> 'on' | 'off'
'PositionTargetPriority' -> 'High' | 'Low'

% Parametros VALIDOS para Planar Joint (prefijo por eje):
'PxTorqueActuationMode'  -> 'InputTorque' | 'NoTorque'
'PxMotionActuationMode'  -> 'ComputedMotion'
'PxSensePosition'        -> 'on' | 'off'
'PxSenseVelocity'        -> 'on' | 'off'
'PxDampingCoefficient'   -> '0'
% Idem para Py* y Rz*

% Parametros de File Solid:
'ExtGeomFileName'  -> ruta al STL (NO 'FileName')
'UnitType'         -> 'Custom' (necesario para que lea ExtGeomFileUnits)
'ExtGeomFileUnits' -> 'm' | 'mm' | 'cm' | etc.

% INVALIDOS (causan error):
'SimscapeLogType'                       % usar SimulationInput.setModelParameter
'MotionActuationMode','NoMotion'        % no existe
'MotionActuationMode','ProvidedMotion'  % causa loop algebraico
'FileName'                              % es ExtGeomFileName

% Patron correcto para simular con logging:
simIn = Simulink.SimulationInput(modelName);
simIn = simIn.setModelParameter('SimscapeLogType','all','SimscapeLogName','simlog');
out   = sim(simIn);

% Conexion SL2PS -> joint torque:
add_line(mdl, 'SL2PS_nombre/RConn1', 'Joint_nombre/LConn2')
```

---

## 12. VARIABLES DE WORKSPACE REQUERIDAS

Antes de que `SimscapeConstruct.m` corra, estas variables deben existir en el workspace:

```matlab
% De params.m:
M, m, r, d, l, g, Icy, Icz, Icx, Iw, Iwz, alm
body_W, body_D, body_H
M11, M12, M22, M33, det0

% Del escenario (LQR_Con_SegVelocidad.m):
modelName            % 'Segway_Testbench_v5'
t_sim, theta0_deg, alpha0_deg
tau_rider, th_lean, t_lean, t_back
tau_alpha, alpha_ref_deg, t_avance_a, t_stop_a
Kv, v_max, int_max
K1a, K2              % ganancias LQR calculadas
V_sat_a, V_sat_d, V_sat_f
```

Si falta alguna -> `SimscapeConstruct.m` falla con "Undefined variable". Solucion: correr `params.m` y luego el script principal completo.

---

## 13. COMO INTERPRETAR EL DEBUG_LOG

Archivo: `Debug/debug_log.txt` — generado al final de cada run.

**Columnas:** `Evento | th_ODE | th_SM | dx_ODE | dx_SM | al_ODE | al_SM | yw_ODE | yw_SM | slip`
- `th` = theta (pitch) en grados
- `dx` = velocidad avance m/s
- `al` = alpha (yaw) en grados
- `yw` = desplazamiento lateral en metros
- `slip` = siempre 0 (restriccion lateral virtual lo garantiza)

**Valores esperados en un run sano (alpha_ref=30 deg):**

| Evento | th_SM | dx_SM | al_SM |
|--------|-------|-------|-------|
| t=0 | 0 deg | 0 m/s | 0 deg |
| t_giro (t=8s) | ~-0.5 deg | ~0.7 m/s | ~0 deg |
| t_recto (t=12s) | ~-0.1 deg | ~0.5 m/s | **~28-29 deg** |
| t_back (t=13s) | ~-0.1 deg | ~0.5 m/s | **~18 deg** |
| t_fin (t=20s) | ~0.3 deg | ~0 m/s | **~0 deg** |

**Senales de alarma:**
- `al_SM` en t_fin > 5 deg -> yaw no converge -> revisar ForceDecomp / K2
- `th_SM` en t_fin > 2 deg -> balance no converge -> revisar K1a / V_sat
- `al_SM` en t_recto < 20 deg (con alpha_ref=30 deg) -> yaw lento -> revisar k_lat o subir Q2
- `yw_SM` muy diferente de `yw_ODE` -> restriccion lateral insuficiente -> subir k_lat

---

## 14. ARBOL DE DIAGNOSTICO RAPIDO

```
El script falla al construir el modelo?
  |-- "Undefined variable"          -> correr params.m primero
  |-- Error en port handles         -> hacer SimulationCommand='update' antes de get_param
  |-- "block already exists"        -> cerrar MATLAB y reiniciar (bdclose all no fue suficiente)
  '-- "unit could not be read"      -> falta UnitType='Custom' en File Solid

La simulacion termina pero alpha no responde?
  |-- Verificar que Planar Joint NO tenga PositionTargetSpecify en Rz
  |-- Verificar que pH_Falpha (no pH_Saref) este conectado a LQR Inport(8)
  '-- Verificar Gain_tau_al = d*alm/r conectado a Planar Joint Rz (LConn4)

Alpha responde pero va lento (Simscape << ODE)?
  |-- K2 insuficiente -> subir Q2(1,1) (actualmente 1800)
  |-- tau_alpha demasiado grande -> bajar de 1.2s
  '-- k_lat demasiado alto -> baja la maniobrabilidad

Theta no converge (segway cae)?
  |-- Verificar K1a tiene 4 elementos [theta_err, dtheta, dx, int_ev]
  |-- Verificar Gain_F = 2*alm/r conectado a ForceDecomp
  '-- Verificar que Joint_theta NO tenga TorqueActuationMode='InputTorque'

Robot deriva lateralmente en 3D?
  |-- k_lat insuficiente -> subir de 10000
  '-- ForceDecomp no recibe vx, vy, alpha -> verificar conexiones

Las graficas fallan / error en seccion try?
  |-- out.get('al_log') falla -> simulacion no termino, revisar mensaje [Sim]
  '-- Dimensiones no coinciden en plot -> revisar SaveFormat='Array' en ToWS
```

---

## 15. REGLAS PARA AGENTES / MODIFICACIONES

**Antes de cambiar cualquier cosa:**
1. Leer `params.m` — todos los parametros fisicos estan ahi, no hardcodear valores
2. Leer la seccion de bugs (#9) — varios errores ya se cometieron y corrigieron
3. `Segway_Testbench_v5.slx` se regenera en cada run — no editarlo a mano
4. `segway_modelo_resultados.mat` es cache — si se corrompe, borrar y re-correr `ModelKane_final.m`
5. `grid_floor.stl` se regenera en cada run — no editarlo a mano

**Al modificar ganancias LQR:**
- Cambiar `Q1a` o `Q2` en `LQR_Con_SegVelocidad.m` seccion 2
- El `LQR_Controller` en Simscape se regenera automaticamente con los nuevos K
- Verificar polos resultantes (todos con parte real < 0)

**Al modificar SimscapeConstruct.m:**
- Siempre hacer `set_param(modelName,'SimulationCommand','update')` antes de leer port handles
- Planar Joint: `numel(pH.RConn) == 7`, `numel(pH.LConn) == 4`
- Revolute con sensing: `numel(pH.RConn) == 3`
- Pasar `PositionTargetValue` siempre en grados, nunca en radianes
- File Solid: usar `ExtGeomFileName` (no `FileName`), con `UnitType='Custom'`

**No tocar sin entender el contexto completo:**
- `ForceDecomp` — combina fuerza de avance + restriccion lateral. Si se quita, el robot deriva en curvas
- El integrador `Int_ev` con `+-int_max` — es el anti-windup del lean-to-speed
- La ganancia `Gain_F = 2*alm/r` sobre Va — reemplaza el rolling constraint
- `Filtro_alpha` conectado a `LQR Inport(8)` — si se conecta `Sum_aref` directo, el yaw recibe escalon duro
- Estado 9 de la ODE — si se quita, ODE y Simscape filtran alpha_ref diferente -> mismatch

---

## 16. HISTORIAL DE VERSIONES DEL MODELO SIMSCAPE

| Version | Cadena cinematica | Problema |
|---------|-------------------|----------|
| v3 | World -> Joint_alpha(origen) -> RT_ForwardAlign -> Joint_fwd(heading) -> Body | Steiner: M33_eff = M33 + M*x^2. A 4.4m de avance, inercia yaw 163x mayor |
| v4 | World -> Planar Joint(Px,Py,Rz) -> Joint_theta -> Body | Deriva lateral: Planar Joint permite movimiento libre en XY |
| v4.1 | Igual que v4 + ForceDecomp con k_lat=10000 + cuadricula visual | **Version actual.** Funciona correctamente |
