#!/bin/bash
# Script completo para limpiar, compilar y subir firmware + filesystem

PORT=${1:-/dev/ttyUSB0}

echo "=========================================="
echo "  DEPLOY COMPLETO - ESP32 Multi-Sensor"
echo "=========================================="
echo "Puerto: $PORT"
echo ""

# 1. Limpiar build anterior
echo "🧹 Limpiando build anterior..."
pio run --target clean
echo ""

# 2. Compilar firmware
echo "🔨 Compilando firmware..."
pio run
if [ $? -ne 0 ]; then
    echo "❌ Error en la compilación del firmware"
    exit 1
fi
echo "✅ Firmware compilado correctamente"
echo ""

# 2.5. Compilar filesystem
echo "🔨 Compilando filesystem (config.html)..."
pio run --target buildfs
if [ $? -ne 0 ]; then
    echo "❌ Error en la compilación del filesystem"
    exit 1
fi
echo "✅ Filesystem compilado correctamente"
echo ""

# 3. Subir firmware
echo "⬆️ Subiendo firmware..."
echo "   📢 Pon el ESP32 en modo bootloader (BOOT + RESET)"
read -p "   Presiona ENTER cuando estés listo..."
pio run --target upload --upload-port "$PORT"
if [ $? -ne 0 ]; then
    echo "❌ Error subiendo firmware"
    exit 1
fi
echo "✅ Firmware subido correctamente"
echo ""

# 4. Subir filesystem
echo "⬆️ Subiendo filesystem (config.html)..."
echo "   📢 Mantén el ESP32 en modo bootloader"
pio run --target uploadfs --upload-port "$PORT"
if [ $? -ne 0 ]; then
    echo "❌ Error subiendo filesystem"
    exit 1
fi
echo "✅ Filesystem subido correctamente"
echo ""

echo "=========================================="
echo "✅ DEPLOY COMPLETADO"
echo "=========================================="
echo "Presiona RESET en el ESP32"
echo ""
