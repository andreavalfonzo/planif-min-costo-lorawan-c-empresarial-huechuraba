# Ubicación de gateways LoRaWAN — Smart Parking Ciudad Empresarial

Proyecto semestral TEL343 – Planificación y Dimensionamiento de Redes de Computadores (USM).

Calcula el **número mínimo de gateways LoRaWAN** y su ubicación para cubrir todos los
estacionamientos de Ciudad Empresarial (Huechuraba), exigiendo que cada sensor quede cubierto
por al menos `k` gateways. El problema se formula como un *set covering* con programación
lineal entera binaria (ILP) y se compara con una heurística voraz.

## Archivos

| Archivo | Descripción |
|---|---|
| `kml_a_csv.py` | Convierte el mapa de Google My Maps (KMZ/KML) en `sensores.csv` y `sitios.csv` |
| `ilp_gateways.py` | Presupuesto de enlace, modelo ILP, heurística voraz y resultados |
| `sensores.csv` | Un sensor por estacionamiento: `id,x,y` |
| `sitios.csv` | Sitios candidatos para gateways: `id,x,y,costo` |
| `resultado_sensores.csv` | Salida: gateway primario, SF y cobertura de cada sensor |
| `mapa_solucion.png` | Salida: mapa de sensores (por SF), sitios candidatos y gateways elegidos |

Las coordenadas `x`, `y` están en metros, en UTM zona 19 Sur (EPSG:32719).

## Instalación

Requiere Python 3.9 o superior.

```bash
python -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

El código funciona con PuLP 2.x (que trae el solver CBC) y con PuLP 4.x (que usa HiGHS,
instalado mediante `highspy`).

## Uso

### 1. Preparar el mapa en Google My Maps

1. Crear un mapa en <https://mymaps.google.com> con vista satelital.
2. Capa **Estacionamientos**: trazar una línea a lo largo de cada fila de estacionamientos.
3. Capa **sitios**: un marcador en cada lugar candidato para un gateway (azotea, poste,
   fachada). En la descripción, indicar el backhaul: `fibra`/`ethernet` (costo 1.0) o
   `4g`/`5g`/`celular` (costo 1.3). Sin descripción se usa el costo por defecto.
4. Menú ⋮ → *Exportar a KML/KMZ*.

### 2. Generar los CSV

```bash
python kml_a_csv.py Taller4-SmartParking.kmz
```

Opciones:

- `--espaciado 2.5`: metros por estacionamiento. Usar 2.5 para estacionamientos en batería y
  alrededor de 5.5 para estacionamientos en paralelo a la vereda.
- `--costo 1.0`: costo relativo de los sitios sin descripción.

### 3. Resolver el ILP

```bash
python ilp_gateways.py
```

Si `sensores.csv` y `sitios.csv` existen en la carpeta, se usan; si no, el script genera datos
sintéticos para probar el modelo.

## Modelo

Variables: `x_j = 1` si se instala un gateway en el sitio `j`; `y_ij = 1` si el sensor `i`
tiene al sitio `j` como gateway primario.

```
min   Σ_j c_j·x_j
s.a.  Σ_{j∈G(i)} x_j ≥ k                     ∀ i   (k-cobertura)
      Σ_{j∈G(i)} y_ij = 1                    ∀ i   (un gateway primario)
      y_ij ≤ x_j                             ∀ i, j∈G(i)
      Σ_i λ_i·y_ij ≤ Cap·x_j                 ∀ j   (capacidad del gateway)
      x_j, y_ij ∈ {0,1}
```

- `G(i)`: sitios cuyo presupuesto de enlace con el sensor `i` cumple
  `P_rx ≥ S_SF + M` para algún SF ≤ `SF_MAX`, con
  `P_rx = P_tx + G_tx + G_rx − PL(d) − L_suelo` y un modelo de pérdida log-distancia.
- `k`: número mínimo de gateways que cubren cada sensor (`k = 2` da redundancia).
- El objetivo incluye además un término de desempate muy pequeño que asigna cada sensor al
  gateway con menor SF; no altera el número de gateways.

## Parámetros

Están al inicio de `ilp_gateways.py`. Son **valores de referencia** y deben calibrarse con
la campaña de medición en terreno:

| Parámetro | Valor | Significado |
|---|---|---|
| `P_TX` | 14 dBm | Potencia de transmisión del sensor |
| `G_RX` | 5 dBi | Ganancia de antena del gateway |
| `L_SUELO` | 10 dB | Pérdida adicional por sensor a nivel de suelo |
| `MARGEN` | 10 dB | Margen por sombreamiento/desvanecimiento |
| `N_EXP` | 3.5 | Exponente de pérdida del modelo log-distancia |
| `SF_MAX` | 10 | SF máximo permitido (cuida la batería) |
| `EVENTOS_DIA`, `KEEPALIVE_DIA` | 12, 24 | Mensajes por sensor al día |
| `CAP_GW` | 40 000 | Mensajes/día admitidos por gateway |
| `K` | 2 | Cobertura mínima por sensor |

## Resultado con el mapa actual

Con 861 sensores (60 filas, 2.211 m) y 6 sitios candidatos con backhaul 5G:

- `k = 2`: **3 gateways**, cobertura mínima 2 y media 2,45 gateways por sensor;
  78% de los sensores en SF7.
- `k = 1`: 2 gateways.

## Limitaciones

- El modelo de propagación no considera edificios ni terreno; debe calibrarse con mediciones.
- La probabilidad de colisión (PDR) y la vida útil de batería no están dentro del ILP, porque
  no son lineales. Se verifican después, por cálculo o simulación (ns-3, LoRaSim).