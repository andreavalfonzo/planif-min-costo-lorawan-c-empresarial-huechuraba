"""
Ubicación de gateways LoRaWAN de mínimo costo (set covering con k-cobertura)
Proyecto TEL343 - Smart parking Ciudad Empresarial

Modelo (ILP binario):
    min   sum_j c_j * x_j
    s.a.  sum_{j in G(i)} x_j >= k                  para todo sensor i   (k-cobertura)
          sum_{j in G(i)} y_ij = 1                  para todo sensor i   (gateway primario)
          y_ij <= x_j                               para todo i, j in G(i)
          sum_{i: j in G(i)} lambda_i * y_ij <= Cap_j * x_j   para todo sitio j (carga)
          x_j, y_ij en {0,1}

    G(i) = sitios j cuyo presupuesto de enlace con el sensor i cumple
           P_rx(i,j) >= S_SF + M  para algún SF permitido (SF <= SF_MAX).

Datos de entrada:
    - Si existen sensores.csv (id,x,y) y sitios.csv (id,x,y,costo) se usan.
      Coordenadas en metros (proyección local, p. ej. UTM 19S desde QGIS).
    - Si no existen, se generan datos sintéticos para probar el modelo.

Requisitos: pip install pulp numpy matplotlib
            (con PuLP 4.x además: pip install highspy)
"""

import csv
import math
import os
import random

import numpy as np
import pulp

# ---------------------------------------------------------------------------
# 1. PARÁMETROS (valores de referencia: calibrar con mediciones en terreno)
# ---------------------------------------------------------------------------
SEED = 7

# Radio
P_TX = 14.0          # dBm, potencia de transmisión del sensor
G_TX = 0.0           # dBi, antena del sensor
G_RX = 5.0           # dBi, antena del gateway
L_SUELO = 10.0       # dB, pérdida extra por sensor a nivel de suelo (medir)
MARGEN = 10.0        # dB, margen por sombreamiento/desvanecimiento (M)

# Modelo log-distancia: PL(d) = PL0 + 10*n*log10(d/d0)
PL0 = 31.7           # dB a d0 = 1 m, 915 MHz (espacio libre)
N_EXP = 3.5          # exponente de pérdida urbano (calibrar)

# Sensibilidad típica de transceptor LoRa, BW = 125 kHz [dBm]
SENSIBILIDAD = {7: -123.0, 8: -126.0, 9: -129.0, 10: -132.0, 11: -134.5, 12: -137.0}
SF_MAX = 10          # se limitan SF altos para cuidar la batería (5 años)

# Tráfico y capacidad
EVENTOS_DIA = 12     # cambios de ocupación por estacionamiento al día (medir)
KEEPALIVE_DIA = 24   # mensajes de supervisión al día
CAP_GW = 40000       # mensajes/día que se admiten por gateway (cota conservadora)

K = 2                # número mínimo de gateways que deben cubrir cada sensor

# Datos sintéticos (si no hay CSV)
ANCHO, ALTO = 1400, 1000     # m, extensión aproximada del área de estudio
N_SENSORES = 1500
N_SITIOS = 30


# ---------------------------------------------------------------------------
# 2. DATOS
# ---------------------------------------------------------------------------
def leer_csv(ruta, campos):
    with open(ruta, newline="", encoding="utf-8") as f:
        return [{c: (row[c] if c == "id" else float(row[c])) for c in campos}
                for row in csv.DictReader(f)]


def datos_sinteticos():
    rng = random.Random(SEED)
    # Estacionamientos agrupados en "playas" de superficie y tramos de calle
    centros = [(rng.uniform(100, ANCHO - 100), rng.uniform(100, ALTO - 100)) for _ in range(12)]
    sensores = []
    for i in range(N_SENSORES):
        cx, cy = rng.choice(centros)
        sensores.append({"id": f"S{i}",
                         "x": min(max(rng.gauss(cx, 60), 0), ANCHO),
                         "y": min(max(rng.gauss(cy, 40), 0), ALTO)})
    sitios = []
    for j in range(N_SITIOS):
        tiene_fibra = rng.random() < 0.6
        costo = 1.0 + (0.0 if tiene_fibra else 0.3)   # costo relativo (GW + backhaul)
        sitios.append({"id": f"G{j}", "x": rng.uniform(0, ANCHO),
                       "y": rng.uniform(0, ALTO), "costo": costo})
    return sensores, sitios


if os.path.exists("sensores.csv") and os.path.exists("sitios.csv"):
    sensores = leer_csv("sensores.csv", ["id", "x", "y"])
    sitios = leer_csv("sitios.csv", ["id", "x", "y", "costo"])
    print("Datos leídos desde CSV")
else:
    sensores, sitios = datos_sinteticos()
    print("Usando datos sintéticos")

I = range(len(sensores))
J = range(len(sitios))
carga = EVENTOS_DIA + KEEPALIVE_DIA      # lambda_i, mensajes/día por sensor


# ---------------------------------------------------------------------------
# 3. PRESUPUESTO DE ENLACE -> conjuntos G(i)
# ---------------------------------------------------------------------------
def perdida(d):
    return PL0 + 10 * N_EXP * math.log10(max(d, 1.0))


def sf_minimo(p_rx):
    """Menor SF que cumple P_rx >= S_SF + M; None si ninguno <= SF_MAX."""
    for sf in range(7, SF_MAX + 1):
        if p_rx >= SENSIBILIDAD[sf] + MARGEN:
            return sf
    return None


sf_ij = {}          # (i, j) -> SF mínimo que funciona
G = {i: [] for i in I}
for i in I:
    for j in J:
        d = math.dist((sensores[i]["x"], sensores[i]["y"]), (sitios[j]["x"], sitios[j]["y"]))
        p_rx = P_TX + G_TX + G_RX - perdida(d) - L_SUELO
        sf = sf_minimo(p_rx)
        if sf is not None:
            G[i].append(j)
            sf_ij[i, j] = sf

sin_cobertura = [sensores[i]["id"] for i in I if len(G[i]) < K]
if sin_cobertura:
    print(f"ADVERTENCIA: {len(sin_cobertura)} sensores tienen menos de {K} sitios candidatos.")
    print("  El problema es infactible: agregue sitios candidatos o revise parámetros.")
    print("  Ejemplos:", sin_cobertura[:10])
    raise SystemExit(1)


# ---------------------------------------------------------------------------
# 4. MODELO ILP
# ---------------------------------------------------------------------------
prob = pulp.LpProblem("ubicacion_gateways", pulp.LpMinimize)


def binaria(nombre):
    """Crea una variable binaria; compatible con PuLP 2.x y 4.x."""
    if hasattr(prob, "add_variable"):                 # PuLP >= 4
        return prob.add_variable(nombre, cat="Binary")
    return pulp.LpVariable(nombre, cat="Binary")      # PuLP 2.x


x = {j: binaria(f"x_{j}") for j in J}
y = {(i, j): binaria(f"y_{i}_{j}") for i in I for j in G[i]}

# Objetivo: costo de gateways + desempate muy pequeño que asigna cada sensor
# al gateway con menor SF (mejor enlace). EPS es tan chico que nunca cambia
# el número de gateways; solo ordena la asignación primaria.
EPS = 0.01 / (len(I) * 12)
prob += (pulp.lpSum(sitios[j]["costo"] * x[j] for j in J)
         + EPS * pulp.lpSum(sf_ij[i, j] * y[i, j] for (i, j) in y))

for i in I:
    prob += pulp.lpSum(x[j] for j in G[i]) >= K, f"cobertura_{i}"
    prob += pulp.lpSum(y[i, j] for j in G[i]) == 1, f"asignacion_{i}"
    for j in G[i]:
        prob += y[i, j] <= x[j], f"enlace_{i}_{j}"

for j in J:
    prob += (pulp.lpSum(carga * y[i, j] for i in I if (i, j) in y) <= CAP_GW * x[j],
             f"capacidad_{j}")

if hasattr(pulp, "PULP_CBC_CMD"):                 # PuLP 2.x trae CBC incluido
    solver = pulp.PULP_CBC_CMD(msg=False, timeLimit=300)
else:                                             # PuLP 4.x usa HiGHS (pip install highspy)
    solver = pulp.HiGHS(msg=False, timeLimit=300)
res = prob.solve(solver)
if hasattr(pulp, "LpStatus"):                     # PuLP 2.x
    estado = pulp.LpStatus[prob.status]
else:                                             # PuLP 4.x
    estado = res.status_str
print(f"\nEstado del solver: {estado}")
if estado != "Optimal":
    raise SystemExit("No se encontró solución óptima.")

elegidos = [j for j in J if x[j].value() > 0.5]
costo_ilp = sum(sitios[j]["costo"] for j in elegidos)


# ---------------------------------------------------------------------------
# 5. HEURÍSTICA VORAZ (línea base de comparación)
# ---------------------------------------------------------------------------
def voraz():
    falta = {i: K for i in I}               # coberturas que le faltan a cada sensor
    cubre = {j: [i for i in I if j in G[i]] for j in J}
    abiertos = set()
    while any(f > 0 for f in falta.values()):
        mejor, mejor_razon = None, 0.0
        for j in J:
            if j in abiertos:
                continue
            ganancia = sum(1 for i in cubre[j] if falta[i] > 0)
            razon = ganancia / sitios[j]["costo"]
            if razon > mejor_razon:
                mejor, mejor_razon = j, razon
        if mejor is None:
            return None
        abiertos.add(mejor)
        for i in cubre[mejor]:
            falta[i] = max(0, falta[i] - 1)
    return sorted(abiertos)


elegidos_voraz = voraz()
costo_voraz = sum(sitios[j]["costo"] for j in elegidos_voraz)


# ---------------------------------------------------------------------------
# 6. RESULTADOS
# ---------------------------------------------------------------------------
primario = {i: next(j for j in G[i] if y[i, j].value() > 0.5) for i in I}
sf_sensor = {i: min(sf_ij[i, j] for j in G[i] if j in elegidos) for i in I}
cobertura = {i: sum(1 for j in G[i] if j in elegidos) for i in I}

print(f"\n--- ILP (óptimo) ---")
print(f"Gateways instalados: {len(elegidos)}  |  costo relativo: {costo_ilp:.2f}")
for j in elegidos:
    n_asig = sum(1 for i in I if primario[i] == j)
    print(f"  {sitios[j]['id']:>4}  ({sitios[j]['x']:7.1f}, {sitios[j]['y']:7.1f})  "
          f"sensores primarios: {n_asig:4d}  carga: {n_asig * carga:6d} msg/día")

print("\nDistribución de SF asignado:")
for sf in range(7, SF_MAX + 1):
    n = sum(1 for i in I if sf_sensor[i] == sf)
    print(f"  SF{sf}: {n:5d} sensores ({100 * n / len(I):5.1f} %)")

print(f"\nCobertura mínima / media por sensor: {min(cobertura.values())} / "
      f"{np.mean(list(cobertura.values())):.2f} gateways")

print(f"\n--- Voraz (línea base) ---")
print(f"Gateways instalados: {len(elegidos_voraz)}  |  costo relativo: {costo_voraz:.2f}")
print(f"Brecha voraz vs. óptimo: {100 * (costo_voraz - costo_ilp) / costo_ilp:.1f} %")

with open("resultado_sensores.csv", "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["sensor", "x", "y", "gateway_primario", "sf", "n_gateways_cubren"])
    for i in I:
        w.writerow([sensores[i]["id"], f"{sensores[i]['x']:.1f}", f"{sensores[i]['y']:.1f}",
                    sitios[primario[i]]["id"], sf_sensor[i], cobertura[i]])
print("\nAsignación por sensor guardada en resultado_sensores.csv")

# Mapa (opcional)
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 6))
    colores = {7: "tab:green", 8: "tab:olive", 9: "tab:orange", 10: "tab:red",
               11: "tab:purple", 12: "black"}
    for sf in range(7, SF_MAX + 1):
        pts = [(sensores[i]["x"], sensores[i]["y"]) for i in I if sf_sensor[i] == sf]
        if pts:
            ax.scatter(*zip(*pts), s=4, c=colores[sf], label=f"Sensor SF{sf}")
    ax.scatter([s["x"] for s in sitios], [s["y"] for s in sitios], marker="^",
               facecolors="none", edgecolors="gray", s=60, label="Sitio candidato")
    ax.scatter([sitios[j]["x"] for j in elegidos], [sitios[j]["y"] for j in elegidos],
               marker="^", c="tab:blue", s=120, label="Gateway instalado")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Solución ILP: {len(elegidos)} gateways (k = {K})")
    leg = ax.legend(loc="upper right", fontsize=8)
    for h in getattr(leg, "legend_handles", getattr(leg, "legendHandles", [])):
        h.set_sizes([40])
    ax.set_aspect("equal")
    ax.ticklabel_format(useOffset=False, style="plain")
    fig.tight_layout()
    fig.savefig("mapa_solucion.png", dpi=200)
    print("Mapa guardado en mapa_solucion.png")
except ImportError:
    pass
