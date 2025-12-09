#!/bin/bash

# Script para desplegar actualizaciones OTA
# Uso: ./deploy_script.sh 1.1.0

VERSION=${1:-"1.0.0"}
SERVER_URL="http://ota.boisolo.com/multi-sensor-iot"
# Ajusta esta ruta según la configuración de tu servidor ota.boisolo.com
SERVER_PATH="/var/www/html/ota.boisolo.com/multi-sensor-iot"

echo "🚀 Desplegando versión $VERSION"

# 1. Verificar que el firmware existe
FIRMWARE_FILE=".pio/build/esp32dev/firmware.bin"
if [ ! -f "$FIRMWARE_FILE" ]; then
    echo "❌ Error: No se encontró el firmware. Ejecuta 'pio run' primero."
    exit 1
fi

# 2. Crear archivo versionado
VERSIONED_FIRMWARE="multi-sensor-iot-$VERSION.bin"
cp "$FIRMWARE_FILE" "$VERSIONED_FIRMWARE"
echo "✅ Copiado firmware a $VERSIONED_FIRMWARE"

# 3. Subir firmware al servidor
echo "📤 Subiendo firmware al servidor ota.boisolo.com..."
scp "$VERSIONED_FIRMWARE" user@ota.boisolo.com:"$SERVER_PATH/$VERSIONED_FIRMWARE"
if [ $? -eq 0 ]; then
    echo "✅ Firmware subido exitosamente"
else
    echo "❌ Error al subir firmware"
    exit 1
fi

# 4. Crear/actualizar version.json
cat > version.json << EOF
{
  "version": "$VERSION",
  "url": "$SERVER_URL/$VERSIONED_FIRMWARE",
  "checksum": "sha256:$(sha256sum "$VERSIONED_FIRMWARE" | cut -d' ' -f1)",
  "mandatory": false,
  "release_notes": "Multi-Sensor IoT Universal v$VERSION - Despliegue automático $(date)"
}
EOF

# 5. Subir version.json al servidor
echo "📤 Subiendo version.json..."
scp version.json user@ota.boisolo.com:"$SERVER_PATH/version.json"
if [ $? -eq 0 ]; then
    echo "✅ version.json actualizado"
else
    echo "❌ Error al subir version.json"
    exit 1
fi

# 6. Verificar que los archivos están accesibles
echo "🔍 Verificando acceso..."
if curl -s "$SERVER_URL/version.json" > /dev/null; then
    echo "✅ version.json accesible"
else
    echo "❌ Error: version.json no accesible"
    exit 1
fi

if curl -s "$SERVER_URL/$VERSIONED_FIRMWARE" > /dev/null; then
    echo "✅ Firmware accesible"
else
    echo "❌ Error: Firmware no accesible"
    exit 1
fi

# 7. Limpiar archivos temporales
rm -f "$VERSIONED_FIRMWARE" version.json
echo "🧹 Limpieza completada"

echo ""
echo "🎉 Despliegue completado exitosamente!"
echo "📋 Resumen:"
echo "   - Versión: $VERSION"
echo "   - URL: $SERVER_URL/$VERSIONED_FIRMWARE"
echo "   - JSON: $SERVER_URL/version.json"
echo ""
echo "⏰ Los dispositivos comenzarán a actualizarse en los próximos 5 minutos"