#!/bin/bash

# Script para desplegar actualizaciones OTA con incremento automático de versión
# Uso: ./deploy_script_ftp.sh
# La versión se incrementa automáticamente

SERVER_URL="http://ota.boisolo.com/multi-sensor-iot"
SERVER_USER="otaboisolo"
SERVER_HOST="ota.boisolo.com"
SERVER_PASSWORD="@BSLota2026"
REMOTE_PATH="multi-sensor-iot"
TRANSFER_METHOD="lftp"

# 1. Obtener versión actual desde platformio.ini (solo primera línea)
CURRENT_VERSION=$(grep "FW_VERSION=" platformio.ini | head -1 | sed 's/.*FW_VERSION="*\([^"]*\)"*/\1/')

# Limpiar posibles caracteres extra y escapes
CURRENT_VERSION=$(echo "$CURRENT_VERSION" | sed 's/\\//g' | tr -d ' \r\n')

if [ -z "$CURRENT_VERSION" ]; then
    echo "❌ Error: No se pudo leer la versión actual de platformio.ini"
    exit 1
fi

echo "📍 Versión actual: $CURRENT_VERSION"

# 2. Incrementar versión (X.Y.Z -> incrementa Z)
IFS='.' read -r MAJOR MINOR PATCH <<< "$CURRENT_VERSION"
NEW_PATCH=$((PATCH + 1))
NEW_VERSION="$MAJOR.$MINOR.$NEW_PATCH"

echo "📍 Nueva versión: $NEW_VERSION"
echo ""

# 0. Clean ANTES de modificar archivos (evita errores de PlatformIO)
echo "🧹 Limpiando build anterior..."
pio run --target clean
echo "✅ Clean completado"
echo ""

# 3. Actualizar platformio.ini
echo "📝 Actualizando platformio.ini..."
sed -i "s/-D FW_VERSION=\\\"[^\\\"]*\\\"/-D FW_VERSION=\\\"$NEW_VERSION\\\"/" platformio.ini
echo "✅ Versión actualizada en platformio.ini"

# 4. Actualizar comentarios en multi-sensor-iot.ino
echo "📝 Actualizando multi-sensor-iot.ino..."
sed -i "s/v1\.[0-9]\+\.[0-9]\+/v$NEW_VERSION/g" src/multi-sensor-iot.ino
echo "✅ Versión actualizada en multi-sensor-iot.ino"
echo ""

# 6. Compilar
echo "🔨 Compilando firmware..."
pio run
if [ $? -ne 0 ]; then
    echo "❌ Error en la compilación"
    exit 1
fi
echo "✅ Compilación exitosa"
echo ""

echo "=========================================="
echo "  DEPLOY OTA v$NEW_VERSION"
echo "=========================================="
echo ""

# 7. Verificar que el firmware existe
FIRMWARE_FILE=".pio/build/esp32dev/firmware.bin"
if [ ! -f "$FIRMWARE_FILE" ]; then
    echo "❌ Error: No se encontró el firmware."
    exit 1
fi

# 8. Verificar conexión
echo "🔍 Verificando conexión al servidor..."
if ! command -v lftp &> /dev/null; then
    echo "🔐 Instalando lftp..."
    sudo apt-get update && sudo apt-get install -y lftp
fi

# 9. Crear archivo versionado
VERSIONED_FIRMWARE="multi-sensor-iot-$NEW_VERSION.bin"
cp "$FIRMWARE_FILE" "$VERSIONED_FIRMWARE"
echo "✅ Copiado firmware a $VERSIONED_FIRMWARE"

# 10. Subir firmware
echo "📤 Subiendo firmware al servidor..."
UPLOAD_RESULT=$(lftp -u "$SERVER_USER@$SERVER_HOST,$SERVER_PASSWORD" -e "set ssl:verify-certificate no; cd $REMOTE_PATH; put $VERSIONED_FIRMWARE; quit" $SERVER_HOST 2>&1)

if [ $? -eq 0 ]; then
    echo "✅ Firmware subido exitosamente"
    echo "   Tamaño: $(ls -lh "$VERSIONED_FIRMWARE" | awk '{print $5}')"
else
    echo "❌ Error al subir firmware:"
    echo "   $UPLOAD_RESULT"
    exit 1
fi

# 11. Crear/actualizar version.json
cat > version.json << EOF
{
  "version": "$NEW_VERSION",
  "url": "$SERVER_URL/$VERSIONED_FIRMWARE",
  "checksum": "sha256:$(sha256sum "$VERSIONED_FIRMWARE" | cut -d' ' -f1)",
  "mandatory": false,
  "release_notes": "Multi-Sensor IoT Universal v$NEW_VERSION - Despliegue automático $(date)"
}
EOF

# 12. Subir version.json
echo "📤 Subiendo version.json..."
JSON_RESULT=$(lftp -u "$SERVER_USER@$SERVER_HOST,$SERVER_PASSWORD" -e "set ssl:verify-certificate no; cd $REMOTE_PATH; put version.json; quit" $SERVER_HOST 2>&1)

if [ $? -eq 0 ]; then
    echo "✅ version.json actualizado"
    echo "   Checksum: $(sha256sum "$VERSIONED_FIRMWARE" | cut -d' ' -f1 | cut -c1-16)..."
else
    echo "❌ Error al subir version.json:"
    echo "   $JSON_RESULT"
    exit 1
fi

# 13. Verificar acceso HTTP
echo "🔍 Verificando acceso HTTP..."
HTTP_STATUS=$(curl -s -o /dev/null -w "%{http_code}" "$SERVER_URL/version.json")
if [ "$HTTP_STATUS" = "200" ]; then
    echo "✅ version.json accesible vía HTTP (200 OK)"
else
    echo "⚠️ version.json no accesible (HTTP $HTTP_STATUS)"
fi

HTTP_STATUS=$(curl -s -o /dev/null -w "%{http_code}" "$SERVER_URL/$VERSIONED_FIRMWARE")
if [ "$HTTP_STATUS" = "200" ]; then
    echo "✅ Firmware accesible vía HTTP (200 OK)"
else
    echo "⚠️ Firmware no accesible (HTTP $HTTP_STATUS)"
fi

# 14. Limpiar archivos temporales
rm -f "$VERSIONED_FIRMWARE" version.json
echo "🧹 Limpieza completada"

echo ""
echo "=========================================="
echo "🎉 DESPLIEGUE COMPLETADO!"
echo "=========================================="
echo "📋 Resumen:"
echo "   - Versión anterior: $CURRENT_VERSION"
echo "   - Nueva versión: $NEW_VERSION"
echo "   - URL: $SERVER_URL/$VERSIONED_FIRMWARE"
echo "   - JSON: $SERVER_URL/version.json"
echo "   - Servidor: $SERVER_HOST"
echo "   - Path: $REMOTE_PATH"
echo ""
echo "⏰ Los dispositivos comenzarán a actualizarse en los próximos 5 minutos"
echo ""
