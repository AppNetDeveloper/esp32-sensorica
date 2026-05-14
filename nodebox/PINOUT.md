# Pinout Rápido - Multi-Sensor IoT

## WT32-ETH01 GPIO Assignments

```
┌─────────────────────────────────────────────────────────────┐
│                    ESP32 WT32-ETH01                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ETHERNET (LAN8720) - NO CAMBIAR                            │
│  ├── GPIO 18: MDIO                                          │
│  ├── GPIO 23: MDC                                           │
│  ├── GPIO 16: PHY POWER                                     │
│  └── GPIO 0:  ETH CLK                                       │
│                                                             │
│  SENSORES                                                   │
│  ├── GPIO 25: TRIG / Pulsador 1 (alt)                       │
│  ├── GPIO 26: ECHO  / Pulsador 2 (alt)                      │
│  ├── GPIO 13: Pulsador 1 (principal)                        │
│  ├── GPIO 14: Pulsador 2 (principal)                        │
│  └── GPIO 32: Sensor Vibración SW-420                       │
│                                                             │
│  CONTROL                                                    │
│  ├── GPIO 12: Botón Config (a GND)                          │
│  ├── GPIO 2:  LED Azul (Config)                             │
│  ├── GPIO 4:  LED Verde (Status OK)                         │
│  └── GPIO 5:  LED Rojo (Error)                              │
│                                                             │
│  ALIMENTACIÓN                                               │
│  ├── VIN:   5V DC                                           │
│  ├── GND:   Tierra                                          │
│  └── 3.3V:  Salida                                          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Sensores Disponibles

| # | Sensor | GPIOs | Conexión |
|---|--------|-------|----------|
| 0 | 📏 Ultrasónico | TRIG: 25, ECHO: 26 | 5V, GND, TRIG, ECHO |
| 1 | 🔘 1 Pulsador | GPIO 13 | Entre GPIO y GND |
| 2 | 🔘🔘 2 Pulsadores | GPIO 13, 14 | Entre GPIO y GND |
| 3 | 📳 Vibración | GPIO 32 | 3.3V, GND, OUT |
| 4 | 📳🔘 Vibración + Pulsador | Vib: 32, Btn: 13 | Ambos conectados |

---

## Conexiones por Sensor

### Sensor Ultrasónico (HC-SR04 / JSN-SR04T)

```
ESP32           Sensor
─────          ──────
5V   ───────►  VCC
GND  ───────►  GND
GPIO 25 ─────► TRIG
GPIO 26 ─────► ECHO
```

### 1 Pulsador

```
ESP32           Pulsador
─────          ─────────
GND  ─────────┐
              │
GPIO 13 ──────┴─►  (Botón entre GPIO y GND)
```

### 2 Pulsadores

```
ESP32           Pulsador 1      Pulsador 2
─────          ─────────      ─────────
GND  ─────────┐               ┌────────┐
              │               │        │
GPIO 13 ──────┴─►  SW        ├────────┤
GPIO 14 ─────────────────────┴─►  SW  │
                             └────────┘
```

### Sensor Vibración (SW-420)

```
ESP32           SW-420
─────          ──────
3.3V ───────►  VCC
GND  ───────►  GND
GPIO 32 ─────► OUT
```

### Vibración + Pulsador

```
ESP32           SW-420          Pulsador
─────          ──────          ─────────
3.3V ───────►  VCC
GND  ───────►  GND    ─────────┐
GPIO 32 ─────► OUT             │
                              ┌┴─►  SW
GPIO 13 ──────────────────────┘
```

---

## LEDs de Estado

| LED | GPIO | Color | Estado |
|-----|------|-------|--------|
| Status | 4 | 🟢 Verde | ON = OK |
| Error | 5 | 🔴 Rojo | Parpadeando = Error |
| Config | 2 | 🔵 Azul | ON = Bridge, Parpadeando = Hotspot |

---

## Botón de Configuración

| GPIO | Acción | Modo |
|------|--------|------|
| 12 (a GND) | 3 segundos | Bridge (Ethernet + WiFi AP) |
| 12 (a GND) | 10 segundos | Hotspot (WiFi solo) |

