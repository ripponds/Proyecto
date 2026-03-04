% =========================================================================
% LINEALIZACIÓN SIMBÓLICA AUTOMÁTICA - MODELO TWIP (Two-Wheeled Inverted Pendulum)
% De Energías Puras a Matrices de Estado (A, B) para diseño LQR
%
% TEORÍA BASE: Mecánica Analítica de Lagrange
%   El Lagrangiano L = T - V define la dinámica completa del sistema.
%   Las ecuaciones de movimiento se obtienen de:
%       d/dt(∂L/∂q̇) - ∂L/∂q = τ
%   donde q son coordenadas generalizadas y τ son fuerzas/torques generalizados.
%
% ESTADOS: X = [x, θ, ψ, ẋ, θ̇, ψ̇]
%   x  = desplazamiento lineal del robot (traslación hacia adelante)
%   θ  = ángulo de inclinación (pitch) — variable crítica para estabilidad
%   ψ  = ángulo de guiñada (yaw) — orientación en plano horizontal
%   ẋ, θ̇, ψ̇ = velocidades generalizadas correspondientes
%
% PARÁMETROS FÍSICOS:
%   mB = masa del chasis (body)          mW = masa de cada rueda
%   r  = radio de la rueda               d  = distancia entre ruedas (track)
%   l  = distancia del eje al CoM        g  = aceleración gravitacional
%   I1 = inercia chasis (eje roll)       I2 = inercia chasis (eje pitch)
%   I3 = inercia chasis (eje yaw)        J  = inercia de cada rueda (spin)
%   K  = inercia combinada motores en yaw
% =========================================================================
clc; clear;

%% -----------------------------------------------------------------------
%  1. DEFINICIÓN DE VARIABLES SIMBÓLICAS
%  -----------------------------------------------------------------------
% Parámetros físicos del robot (todos reales y positivos físicamente)
syms mB mW r d l g I1 I2 I3 J K real

% Variables de estado: posiciones y velocidades generalizadas
syms x th ps dx dth dps real
% Variables de control: torques aplicados por motor izquierdo y derecho
syms uL uR real

% Vector de coordenadas generalizadas q ∈ ℝ³
q  = [x; th; ps];
% Vector de velocidades generalizadas q̇ ∈ ℝ³
dq = [dx; dth; dps];
% Vector de estado completo X = [q; q̇] ∈ ℝ⁶
X  = [q; dq];
% Vector de entradas de control U = [uL; uR] ∈ ℝ²
U  = [uL; uR];

%% -----------------------------------------------------------------------
%  2. ENERGÍA CINÉTICA TRASLACIONAL T_trans
%  -----------------------------------------------------------------------
% La energía cinética traslacional del chasis se calcula a partir de la
% velocidad del Centro de Masa (CoM) del cuerpo en coordenadas cartesianas.
%
% El CoM del chasis está ubicado a distancia 'l' del eje de ruedas.
% Si el robot se inclina un ángulo θ, la posición del CoM es:
%   P_CoM = [x + l·sin(θ),  0,  l·cos(θ)]   (en 2D sagital)
%
% Diferenciando respecto al tiempo, la velocidad del CoM es:
%   ẋ_CoM = ẋ + l·θ̇·cos(θ)       → componente horizontal (avance)
%   ẏ_CoM = l·ψ̇·sin(θ)            → componente lateral (giro yaw)
%   ż_CoM = -l·θ̇·sin(θ)           → componente vertical (subida/bajada CoM)
%
% CORRECCIÓN RESPECTO A VERSIÓN ANTERIOR:
%   Se añade el término (l·θ̇·sin(θ))² que representa la variación
%   de altura del CoM al inclinarse. Aunque se anula en θ=0 (equilibrio),
%   es esencial para que el modelo no lineal sea físicamente correcto.
%
% T_chasis = (1/2)·mB·|v_CoM|²
T_chasis = 0.5 * mB * ( ...
    (dx + l*dth*cos(th))^2 + ...    % (ẋ_CoM)² — avance con inclinación
    (l*dth*sin(th))^2       + ...   % (ż_CoM)² — ← TÉRMINO CORREGIDO: variación altura CoM
    (l*dps*sin(th))^2 );            % (ẏ_CoM)² — movimiento lateral por yaw

% Energía cinética traslacional de las dos ruedas (tratadas como discos rígidos)
% La rueda avanza con velocidad ẋ del robot.
% En guiñada, cada rueda se mueve con velocidad tangencial (d/2)·ψ̇
%   T_ruedas_trans = (1/2)·(2·mW)·[ẋ² + ((d/2)·ψ̇)²]
T_ruedas_trans = 0.5 * (2*mW) * (dx^2 + (dps*d/2)^2);

% Suma de energías traslacionales
T_trans = T_chasis + T_ruedas_trans;

%% -----------------------------------------------------------------------
%  3. ENERGÍA CINÉTICA ROTACIONAL T_rot
%  -----------------------------------------------------------------------
% La energía rotacional del chasis depende de la velocidad angular en
% los tres ejes principales de inercia del cuerpo:
%
%   Eje Roll  (I1): componente de ψ̇ perpendicular al eje de inclinación → ψ̇·sin(θ)
%   Eje Pitch (I2): rotación directa de inclinación → θ̇
%   Eje Yaw   (I3): componente de ψ̇ proyectada sobre eje vertical → ψ̇·cos(θ)
%
%   T_chasis_rot = (1/2)·[I1·(ψ̇·sin θ)² + I2·θ̇² + I3·(ψ̇·cos θ)²]
T_chasis_rot = 0.5 * (I1*(dps*sin(th))^2 + I2*dth^2 + I3*(dps*cos(th))^2);

% Energía cinética de rotación de las ruedas (spin propio sobre su eje):
%   Cada rueda gira a velocidad angular ẋ/r (rodadura pura sin deslizamiento)
%   Para el giro en yaw, cada rueda gira a ±(d/2r)·ψ̇
%   J = inercia de cada rueda respecto a su propio eje
%   Hay 2 ruedas → factor 2 ya incluido en la expresión:
%     T_ruedas_spin = J·[(ẋ/r)² + (d·ψ̇/(2r))²]
T_ruedas_spin = J * (dx^2/r^2 + (dps^2*d^2)/(4*r^2));

% Término K·ψ̇²: inercia adicional de los rotores de los motores al girar
% en yaw. Los rotores tienen masa y al pivotar el robot contribuyen a la
% energía rotacional de guiñada con un momento de inercia efectivo K.
T_motores_yaw = K * dps^2;

% Suma de todas las energías rotacionales
T_rot = T_chasis_rot + T_ruedas_spin + T_motores_yaw;

%% -----------------------------------------------------------------------
%  4. ENERGÍA CINÉTICA TOTAL Y ENERGÍA POTENCIAL
%  -----------------------------------------------------------------------
% Energía cinética total del sistema
T = simplify(T_trans + T_rot);

% Energía potencial: solo el CoM del chasis tiene altura variable.
% Las ruedas ruedan sobre plano horizontal → V_ruedas = cte (se cancela).
%   V = mB·g·l·cos(θ)
%   En θ=0 (robot vertical): V es máxima → punto de equilibrio INESTABLE.
%   La linealización en θ=0 capturará este comportamiento con autovalor real positivo.
V_pot = mB * g * l * cos(th);

%% -----------------------------------------------------------------------
%  5. ECUACIONES DE EULER-LAGRANGE: M·q̈ + C·q̇ + G = τ
%  -----------------------------------------------------------------------
% --- Matriz de Masa M = ∂²T/∂q̇² ---
% M es el Hessiano de T respecto a las velocidades generalizadas.
% Representa la inercia efectiva del sistema en cada dirección generalizada.
% Por construcción es simétrica y definida positiva (T > 0 para q̇ ≠ 0).
M = hessian(T, dq);     % M ∈ ℝ³ˣ³, simétrica, depende de θ

% --- Vector de Fuerzas Gravitacionales G = ∂V/∂q ---
% Gradiente de la energía potencial respecto a coordenadas generalizadas.
% Solo θ aparece en V_pot → G tendrá componente no nula solo en la fila de θ.
G_vec = jacobian(V_pot, q)';    % G ∈ ℝ³

% --- Vector de Momento Generalizado p = ∂T/∂q̇ ---
% El momento generalizado (o cantidad de movimiento generalizada) es el
% gradiente de T respecto a las velocidades. Es el análogo Lagrangiano
% al momento lineal/angular de la mecánica newtoniana.
p_momento = jacobian(T, dq)';   % p ∈ ℝ³

% --- Términos de Coriolis y Centrífugos C·q̇ ---
% Se obtienen de la derivada total del momento generalizado:
%   d/dt(∂T/∂q̇) = (∂²T/∂q̇∂q)·q̇ + M·q̈
% El término de Coriolis/Centrífugos es:
%   C·q̇ = (∂p/∂q)·q̇ - (∂T/∂q)
% donde ∂T/∂q captura los términos centrífugos que emergen del movimiento
V_coriolis = (jacobian(p_momento, q) * dq) - jacobian(T, q)';

%% -----------------------------------------------------------------------
%  6. VECTOR DE FUERZAS GENERALIZADAS τ (Torques de los Motores)
%  -----------------------------------------------------------------------
% Los torques uL y uR de los motores se mapean a las coordenadas
% generalizadas [x, θ, ψ] mediante el principio de trabajo virtual.
%
% Coordenada x (avance):
%   Ambos motores impulsan el robot hacia adelante.
%   La fuerza lineal equivalente es (uL + uR)/r  [N = Nm/m]
%
% Coordenada θ (pitch/inclinación):
%   Al girar ambas ruedas en el mismo sentido, ejercen un par de reacción
%   sobre el chasis que tiende a inclinarlo. El signo negativo refleja la
%   convención: torque positivo en ruedas → inclinación hacia atrás del chasis.
%   τ_θ = -(uL + uR)   [Nm]
%
% Coordenada ψ (yaw/guiñada):
%   La diferencia de torques genera giro: τ_ψ = (d/2r)·(uR - uL)
%   Rueda derecha positiva y rueda izquierda negativa → giro antihorario.
Tau = [ (uL + uR)/r;          % Fuerza generalizada en x    [N]
       -(uL + uR);             % Torque generalizado en θ    [Nm]
        (d/(2*r))*(uR - uL)]; % Torque generalizado en ψ    [Nm]

%% -----------------------------------------------------------------------
%  7. ECUACIONES DE MOVIMIENTO NO LINEALES: q̈ = M⁻¹·(τ - C·q̇ - G)
%  -----------------------------------------------------------------------
% Despejando las aceleraciones generalizadas del sistema de Euler-Lagrange:
%   M·q̈ = τ - C(q,q̇)·q̇ - G(q)
%   q̈   = M⁻¹·[τ - C·q̇ - G]
%
% Esta es la forma explícita de la ODE de segundo orden del robot.
% MATLAB resuelve el sistema lineal M\b en lugar de calcular M⁻¹ explícitamente
% (más eficiente numéricamente, equivalente simbólicamente).
ddq = M \ (Tau - V_coriolis - G_vec);   % q̈ ∈ ℝ³

% Vector de dinámica completa del sistema: ẋ = f(X, U)
%   Forma espacio de estados no lineal de primer orden:
%   d/dt[q; q̇] = [q̇; M⁻¹(τ - C·q̇ - G)]
f_no_lineal = [dq; ddq];    % f ∈ ℝ⁶

%% -----------------------------------------------------------------------
%  8. LINEALIZACIÓN POR JACOBIANO EN EL PUNTO DE EQUILIBRIO
%  -----------------------------------------------------------------------
% TEORÍA DE LINEALIZACIÓN:
%   Para un sistema no lineal ẋ = f(x, u), alrededor del punto de
%   equilibrio (x₀, u₀) donde f(x₀, u₀) = 0, el sistema linealizado es:
%       δẋ = A·δx + B·δu
%   donde:
%       A = ∂f/∂x |_(x₀,u₀)    (Jacobiano respecto al estado)
%       B = ∂f/∂u |_(x₀,u₀)    (Jacobiano respecto a la entrada)
%
% PUNTO DE EQUILIBRIO:
%   x₀ = [x=0, θ=0, ψ=0, ẋ=0, θ̇=0, ψ̇=0]  → Robot vertical, estático
%   u₀ = [uL=0, uR=0]                        → Sin torque aplicado
%
%   Verificación: en θ=0, cos(0)=1, sin(0)=0 → la gravedad actúa
%   axialmente sobre el CoM y el sistema está en equilibrio (inestable).

% Vector de todas las variables a sustituir
eq_vars = [x, th, ps, dx, dth, dps, uL, uR];
% Valores en el punto de equilibrio
eq_vals = [0,  0,  0,  0,  0,  0,  0,  0];

fprintf('Linealizando el sistema por Jacobiano en X₀=[0,0,0,0,0,0]... ');

% Matriz A = ∂f/∂X |_equilibrio  →  Describe la dinámica libre del sistema lineal
% Sus autovalores determinan la estabilidad:
%   Re(λ) < 0 → modos estables    Re(λ) > 0 → modos inestables (como θ)
A_sym = simplify(subs(jacobian(f_no_lineal, X), eq_vars, eq_vals));

% Matriz B = ∂f/∂U |_equilibrio  →  Cómo las entradas afectan al estado
% Columna 1: efecto de uL    Columna 2: efecto de uR
% El par (A,B) debe ser CONTROLABLE para que el LQR tenga solución.
% Verificar: rank(ctrb(A_num, B_num)) == 6  (rango completo)
B_sym = simplify(subs(jacobian(f_no_lineal, U), eq_vars, eq_vals));

fprintf('¡Completado!\n\n');

%% -----------------------------------------------------------------------
%  9. RESULTADOS
%  -----------------------------------------------------------------------
% Las matrices A y B obtenidas son SIMBÓLICAS en función de los parámetros
% físicos del robot. Para usar el LQR:
%   1. Sustituir los parámetros numéricos medidos del robot físico.
%   2. Verificar controlabilidad: rank(ctrb(A,B)) == 6
%   3. Definir matrices de peso Q (estados) y R (control)
%   4. Resolver Riccati: [K,P,e] = lqr(A, B, Q, R)

disp('=================================================================');
disp('   MATRICES DE ESTADO LINEALIZADAS (A, B) PARA DISEÑO LQR       ');
disp('   Sistema: TWIP — Punto de equilibrio: θ=0, reposo, sin torque  ');
disp('=================================================================');

disp(' ');
disp('MATRIZ A [6x6] — Dinámica libre: δẋ = A·δx');
disp('Estructura esperada: bloques [0 I; 0 A22] con A22 con términos de gravedad');
disp(A_sym);

disp(' ');
disp('MATRIZ B [6x2] — Influencia del control: contribución de [uL, uR]');
disp('Columna 1: efecto de uL (motor izquierdo)');
disp('Columna 2: efecto de uR (motor derecho)');
disp(B_sym);

disp('=================================================================');
disp('PRÓXIMOS PASOS PARA LQR:');
disp('  1. Sustituir parámetros numéricos en A_sym y B_sym');
disp('  2. Verificar controlabilidad: rank(ctrb(A,B)) debe ser 6');
disp('  3. Sintonizar Q y R según prioridades de control');
disp('  4. [K,~,~] = lqr(A, B, Q, R)  →  u = -K*(X - X_ref)');
disp('=================================================================');