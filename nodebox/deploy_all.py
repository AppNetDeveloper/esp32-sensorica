#!/usr/bin/env python3
"""
Deploy completo - ESP32 Multi-Sensor
Limpia, compila y sube firmware + filesystem via USB.

Uso:
  python deploy_all.py                  # Puerto por defecto (auto-detecta)
  python deploy_all.py COM3             # Windows
  python deploy_all.py /dev/ttyUSB0     # Linux
  python deploy_all.py /dev/ttyUSB1     # Linux (otro puerto)
"""

import subprocess
import sys
import platform
import os
import serial.tools.list_ports


def find_serial_port():
    """Auto-detecta el primer puerto serial disponible."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        return None
    for p in ports:
        if "USB" in p.description.upper() or "CP210" in p.description or "CH340" in p.description or "UART" in p.description.upper():
            return p.device
    return ports[0].device


def run_step(description, command, ask_confirm=False):
    """Ejecuta un paso del deploy con manejo de errores."""
    print(f"\n{description}")
    if ask_confirm:
        input("   Presiona ENTER cuando estes listo... ")

    result = subprocess.run(command, shell=True)
    if result.returncode != 0:
        print(f"   ERROR: {description} fallo (codigo {result.returncode})")
        sys.exit(1)
    print(f"   OK: {description}")


def main():
    # Determinar puerto serial
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_serial_port()
        if port is None:
            print("ERROR: No se encontro ningun puerto serial conectado.")
            print("Conecta el ESP32 via USB e intenta de nuevo.")
            sys.exit(1)

    system = platform.system()
    is_windows = system == "Windows"

    print("=" * 50)
    print("  DEPLOY COMPLETO - ESP32 Multi-Sensor")
    print("=" * 50)
    print(f"  Sistema: {system}")
    print(f"  Puerto:  {port}")
    print("=" * 50)

    # Cambiar al directorio del proyecto
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    # 1. Limpiar build anterior
    run_step("Limpiando build anterior...", f'"{sys.executable}" -m platformio run --target clean')

    # 2. Compilar firmware
    run_step("Compilando firmware...", f'"{sys.executable}" -m platformio run')

    # 3. Compilar filesystem
    run_step("Compilando filesystem (config.html)...", f'"{sys.executable}" -m platformio run --target buildfs')

    # 4. Subir firmware
    print("\n--- Subiendo firmware ---")
    print("   Pon el ESP32 en modo bootloader (BOOT + RESET)")
    input("   Presiona ENTER cuando este listo... ")
    run_step("Subiendo firmware...", f'"{sys.executable}" -m platformio run --target upload --upload-port {port}')

    # 5. Subir filesystem
    print("\n--- Subiendo filesystem (config.html) ---")
    print("   Manten el ESP32 en modo bootloader")
    run_step("Subiendo filesystem...", f'"{sys.executable}" -m platformio run --target uploadfs --upload-port {port}')

    print("\n" + "=" * 50)
    print("  DEPLOY COMPLETADO")
    print("=" * 50)
    print(f"  Puerto: {port}")
    print("  Presiona RESET en el ESP32")
    print("=" * 50)


if __name__ == "__main__":
    main()
