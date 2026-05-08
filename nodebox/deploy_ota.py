#!/usr/bin/env python3
"""
Script OTA para Multi-Sensor IoT
Incrementa versión automáticamente, compila y sube al servidor
"""

import re
import subprocess
import sys
import os
import json
import hashlib
import datetime
import time

# Configuración
PLATFORMIO_INI = "platformio.ini"
SOURCE_FILE = "src/multi-sensor-iot.ino"
FIRMWARE_PATH = ".pio/build/esp32dev/firmware.bin"
FILESYSTEM_PATH = ".pio/build/esp32dev/littlefs.bin"
SERVER_URL = "http://ota.boisolo.com/multi-sensor-iot"
SERVER_USER = "otaboisolo"
SERVER_HOST = "ota.boisolo.com"
SERVER_PASSWORD = "@BSLota2026"
REMOTE_PATH = "multi-sensor-iot"
PLATFORMIO_ENV = "esp32dev"  # Usar solo este entorno

def get_current_version():
    """Lee la versión actual desde platformio.ini"""
    with open(PLATFORMIO_INI, 'r') as f:
        content = f.read()
        # Buscar FW_VERSION=\"X.Y.Z\" (primera ocurrencia, en esp32dev)
        match = re.search(r'FW_VERSION=\\?"(\d+\.\d+\.\d+)\\?"', content)
        if match:
            return match.group(1)
    return None

def increment_version(version):
    """Incrementa el PATCH version (X.Y.Z -> X.Y.Z+1)"""
    parts = version.split('.')
    parts[2] = str(int(parts[2]) + 1)
    return '.'.join(parts)

def update_version_in_ini(new_version):
    """Actualiza la versión en platformio.ini (ambos entornos)"""
    with open(PLATFORMIO_INI, 'r') as f:
        content = f.read()

    # Reemplazar TODAS las apariciones de FW_VERSION
    content = re.sub(r'FW_VERSION=\\?"\d+\.\d+\.\d+\\?"', f'FW_VERSION=\\"{new_version}\\"', content)

    with open(PLATFORMIO_INI, 'w') as f:
        f.write(content)
    print(f"✅ platformio.ini actualizado a v{new_version}")

def update_version_in_source(new_version):
    """Actualiza las versiones en los comentarios del .ino"""
    with open(SOURCE_FILE, 'r') as f:
        content = f.read()

    # Reemplazar versiones en comentarios v1.X.X.X
    content = re.sub(r'v\d+\.\d+\.\d+', f'v{new_version}', content)

    with open(SOURCE_FILE, 'w') as f:
        f.write(content)
    print(f"✅ multi-sensor-iot.ino actualizado a v{new_version}")

def run_command(cmd, description):
    """Ejecuta un comando y muestra el resultado"""
    print(f"\n{description}...")
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"❌ Error: {result.stderr}")
        return False
    return True

def run_command_live(cmd, description):
    """Ejecuta un comando mostrando salida en tiempo real"""
    print(f"\n{description}...")
    result = subprocess.run(cmd, shell=True)
    return result.returncode == 0

def main():
    print("=" * 50)
    print("  OTA DEPLOY - Multi-Sensor IoT Universal")
    print("=" * 50)

    # 1. Obtener versión actual
    current_version = get_current_version()
    if not current_version:
        print("❌ Error: No se pudo leer la versión actual")
        sys.exit(1)
    print(f"📍 Versión actual: {current_version}")

    # 2. Incrementar versión
    new_version = increment_version(current_version)
    print(f"📍 Nueva versión: {new_version}")

    # 3. Actualizar archivos
    update_version_in_ini(new_version)
    update_version_in_source(new_version)

    # 4. Limpiar cache de PlatformIO (para evitar errores de compilación)
    print("\n🧹 Limpiando build anterior...")
    # Usar pio clean en lugar de rm para evitar problemas
    subprocess.run(f"pio run -e {PLATFORMIO_ENV} --target clean", shell=True)
    print("✅ Clean completado")

    # Pequeño delay para que el sistema de archivos se estabilice
    time.sleep(1)

    # 5. Compilar firmware
    print("\n🔨 Compilando firmware...")
    if not run_command_live(f"pio run -e {PLATFORMIO_ENV}", "Compilando firmware"):
        print("❌ Error en la compilación")
        # Restaurar versión original
        update_version_in_ini(current_version)
        update_version_in_source(current_version)
        sys.exit(1)
    print("✅ Compilación exitosa")

    # 5.5. Compilar filesystem
    print("\n🔨 Compilando filesystem (config.html)...")
    if not run_command_live(f"pio run -e {PLATFORMIO_ENV} --target buildfs", "Compilando filesystem"):
        print("⚠️ Advertencia: Error al compilar filesystem")
    else:
        print("✅ Filesystem compilado")

    # 6. Verificar firmware
    if not os.path.exists(FIRMWARE_PATH):
        print(f"❌ Error: No se encontró {FIRMWARE_PATH}")
        print("ℹ️ Intenta compilar manualmente: pio run -e esp32dev")
        # Restaurar versión original
        update_version_in_ini(current_version)
        update_version_in_source(current_version)
        sys.exit(1)

    firmware_size = os.path.getsize(FIRMWARE_PATH)
    print(f"✅ Firmware encontrado: {firmware_size} bytes")

    # 7. Subir firmware al servidor
    versioned_firmware = f"multi-sensor-iot-{new_version}.bin"
    print(f"\n📤 Subiendo firmware al servidor...")
    print(f"   Origen: {FIRMWARE_PATH}")
    print(f"   Destino: {versioned_firmware}")

    lftp_cmd = f"""lftp -u "{SERVER_USER}@{SERVER_HOST},{SERVER_PASSWORD}" -e "set ssl:verify-certificate no; cd {REMOTE_PATH}; put {FIRMWARE_PATH} -o {versioned_firmware}; quit" {SERVER_HOST}"""

    if not run_command(lftp_cmd, "Subiendo firmware"):
        print("❌ Error subiendo firmware")
        # Restaurar versión original
        update_version_in_ini(current_version)
        update_version_in_source(current_version)
        sys.exit(1)
    print("✅ Firmware subido exitosamente")

    # 7.5. Subir filesystem al servidor OTA
    versioned_filesystem = f"multi-sensor-iot-filesystem-{new_version}.bin"
    print(f"\n📤 Subiendo filesystem al servidor...")
    print(f"   Origen: {FILESYSTEM_PATH}")
    print(f"   Destino: {versioned_filesystem}")

    if os.path.exists(FILESYSTEM_PATH):
        fs_size = os.path.getsize(FILESYSTEM_PATH)
        print(f"   Tamaño: {fs_size} bytes")
        lftp_cmd_fs = f"""lftp -u "{SERVER_USER}@{SERVER_HOST},{SERVER_PASSWORD}" -e "set ssl:verify-certificate no; cd {REMOTE_PATH}; put {FILESYSTEM_PATH} -o {versioned_filesystem}; quit" {SERVER_HOST}"""
        if run_command(lftp_cmd_fs, "Subiendo filesystem"):
            print("✅ Filesystem subido exitosamente")
        else:
            print("⚠️ Error subiendo filesystem, continuando...")
    else:
        print("⚠️ Advertencia: No se encontró el archivo filesystem, omitiendo...")

    # 8. Crear y subir version.json
    print(f"\n📊 Creando version.json...")
    with open(FIRMWARE_PATH, 'rb') as f:
        firmware_data = f.read()
        checksum = "sha256:" + hashlib.sha256(firmware_data).hexdigest()

    version_json = {
        "version": new_version,
        "firmware": f"{SERVER_URL}/{versioned_firmware}",
        "filesystem": f"{SERVER_URL}/{versioned_filesystem}" if os.path.exists(FILESYSTEM_PATH) else None,
        "checksum": checksum,
        "mandatory": False,
        "release_date": datetime.datetime.now().strftime('%Y-%m-%d %H:%M'),
        "changelog": f"Multi-Sensor IoT Universal v{new_version}"
    }

    with open("version.json", 'w') as f:
        json.dump(version_json, f, indent=2)

    print(f"📤 Subiendo version.json...")
    lftp_cmd = f'lftp -u "{SERVER_USER}@{SERVER_HOST},{SERVER_PASSWORD}" -e "set ssl:verify-certificate no; cd {REMOTE_PATH}; put version.json; quit" {SERVER_HOST}'
    if run_command(lftp_cmd, "Subiendo version.json"):
        print("✅ version.json subido")

    # 9. Limpiar
    if os.path.exists("version.json"):
        os.remove("version.json")
    print("\n🧹 Limpieza completada")

    print("\n" + "=" * 50)
    print("🎉 DEPLOY COMPLETADO")
    print("=" * 50)
    print(f"📋 Versión: {current_version} → {new_version}")
    print(f"📋 URL: {SERVER_URL}/{versioned_firmware}")
    print(f"⏰ Los dispositivos se actualizarán en ~30 segundos\n")

if __name__ == "__main__":
    main()
