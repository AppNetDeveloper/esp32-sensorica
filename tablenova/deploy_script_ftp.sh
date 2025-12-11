#!/bin/bash

# Script para desplegar actualizaciones OTA con soporte FTP/SSH
# Uso: ./deploy_script_ftp.sh 1.1.0

VERSION=${1:-"1.0.0"}
SERVER_URL="http://ota.boisolo.com/multi-sensor-iot"

SERVER_USER="otaboisolo"               # Cambia por tu usuario real
SERVER_HOST="ota.boisolo.com"      # Servidor
SERVER_PASSWORD="@BSLota2026"     # Cambia por tu contraseña real
REMOTE_PATH="multi-sensor-iot"  # Ruta relativa desde home en cPanel

# ⚙️ MÉTODO DE TRANSFERENCIA - CAMBIA SEGÚN NECESITES ⚙️
TRANSFER_METHOD="lftp"                # "scp" (SSH) o "ftp" o "lftp"

echo "🚀 Desplegando versión $VERSION usando método: $TRANSFER_METHOD"

# 1. Verificar que el firmware existe
FIRMWARE_FILE=".pio/build/esp32dev/firmware.bin"
if [ ! -f "$FIRMWARE_FILE" ]; then
    echo "❌ Error: No se encontró el firmware. Ejecuta 'pio run' primero."
    exit 1
fi

# 2. Probar conexión y verificar directorio remoto
echo "🔍 Verificando conexión al servidor..."
if ! command -v lftp &> /dev/null; then
    echo "🔐 Instalando lftp..."
    sudo apt-get update && sudo apt-get install -y lftp
fi

echo "📡 Probando conexión FTP con $SERVER_USER@$SERVER_HOST..."
CONNECTION_TEST=$(lftp -u "$SERVER_USER@$SERVER_HOST,$SERVER_PASSWORD" -e "set ssl:verify-certificate no; ls; quit" $SERVER_HOST 2>&1)

if [[ "$CONNECTION_TEST" == *"Access failed"* ]] || [[ "$CONNECTION_TEST" == *"Login failed"* ]] || [[ "$CONNECTION_TEST" == *"not connected"* ]]; then
    echo "❌ ERROR DE CONEXIÓN:"
    echo "   $CONNECTION_TEST"
    echo "   Verifica:"
    echo "   - Usuario: $SERVER_USER@$SERVER_HOST"
    echo "   - Contraseña: [revisar si es correcta]"
    echo "   - Servicio FTP activo en cPanel"
    exit 1
fi

echo "✅ Conexión FTP establecida correctamente"

# 3. Verificar si existe el directorio remoto
echo "📂 Verificando directorio remoto: $REMOTE_PATH"
DIR_TEST=$(lftp -u "$SERVER_USER@$SERVER_HOST,$SERVER_PASSWORD" -e "set ssl:verify-certificate no; cd $REMOTE_PATH; ls; quit" $SERVER_HOST 2>&1)

if [[ "$DIR_TEST" == *"Access failed"* ]] || [[ "$DIR_TEST" == *"No such file or directory"* ]]; then
    echo "⚠️ Directorio no existe. Intentando crearlo..."
    MKDIR_RESULT=$(lftp -u "$SERVER_USER@$SERVER_HOST,$SERVER_PASSWORD" -e "set ssl:verify-certificate no; mkdir $REMOTE_PATH; quit" $SERVER_HOST 2>&1)
    if [[ "$MKDIR_RESULT" == *"mkdir ok"* ]] || [[ "$MKDIR_RESULT" == *"File successfully created"* ]]; then
        echo "✅ Directorio $REMOTE_PATH creado exitosamente"
    else
        echo "❌ Error creando directorio:"
        echo "   $MKDIR_RESULT"
        echo "   Intenta crearlo manualmente via cPanel"
        exit 1
    fi
else
    echo "✅ Directorio $REMOTE_PATH encontrado"
    echo "   Contenido actual:"
    echo "$DIR_TEST" | head -10
fi

# 4. Crear archivo versionado
VERSIONED_FIRMWARE="multi-sensor-iot-$VERSION.bin"
cp "$FIRMWARE_FILE" "$VERSIONED_FIRMWARE"
echo "✅ Copiado firmware a $VERSIONED_FIRMWARE"

# 5. Subir firmware al servidor con verbose
echo "📤 Subiendo firmware al servidor..."
echo "   Origen: $VERSIONED_FIRMWARE"
echo "   Destino: $REMOTE_PATH/$VERSIONED_FIRMWARE"

UPLOAD_RESULT=$(lftp -u "$SERVER_USER@$SERVER_HOST,$SERVER_PASSWORD" -e "set ssl:verify-certificate no; cd $REMOTE_PATH; put $VERSIONED_FIRMWARE; quit" $SERVER_HOST 2>&1)

if [ $? -eq 0 ]; then
    echo "✅ Firmware subido exitosamente"
    echo "   Tamaño: $(ls -lh "$VERSIONED_FIRMWARE" | awk '{print $5}')"
else
    echo "❌ Error al subir firmware:"
    echo "   $UPLOAD_RESULT"
    exit 1
fi

# 6. Crear/actualizar version.json
cat > version.json << EOF
{
  "version": "$VERSION",
  "url": "$SERVER_URL/$VERSIONED_FIRMWARE",
  "checksum": "sha256:$(sha256sum "$VERSIONED_FIRMWARE" | cut -d' ' -f1)",
  "mandatory": false,
  "release_notes": "Multi-Sensor IoT Universal v$VERSION - Despliegue automático $(date)"
}
EOF

# 7. Subir version.json con verbose
echo "📤 Subiendo version.json..."
echo "   Origen: version.json"
echo "   Destino: $REMOTE_PATH/version.json"

JSON_RESULT=$(lftp -u "$SERVER_USER@$SERVER_HOST,$SERVER_PASSWORD" -e "set ssl:verify-certificate no; cd $REMOTE_PATH; put version.json; quit" $SERVER_HOST 2>&1)

if [ $? -eq 0 ]; then
    echo "✅ version.json actualizado"
    echo "   Checksum: $(sha256sum "$VERSIONED_FIRMWARE" | cut -d' ' -f1 | cut -c1-16)..."
else
    echo "❌ Error al subir version.json:"
    echo "   $JSON_RESULT"
    exit 1
fi

# 8. Verificar que los archivos están accesibles vía HTTP
echo "🔍 Verificando acceso HTTP..."
echo "   Verificando: $SERVER_URL/version.json"

HTTP_STATUS=$(curl -s -o /dev/null -w "%{http_code}" "$SERVER_URL/version.json")
if [ "$HTTP_STATUS" = "200" ]; then
    echo "✅ version.json accesible vía HTTP (200 OK)"
else
    echo "⚠️ version.json no accesible (HTTP $HTTP_STATUS)"
    echo "   Puede ser por permisos o propagación DNS"
fi

echo "   Verificando: $SERVER_URL/$VERSIONED_FIRMWARE"
HTTP_STATUS=$(curl -s -o /dev/null -w "%{http_code}" "$SERVER_URL/$VERSIONED_FIRMWARE")
if [ "$HTTP_STATUS" = "200" ]; then
    echo "✅ Firmware accesible vía HTTP (200 OK)"
else
    echo "⚠️ Firmware no accesible (HTTP $HTTP_STATUS)"
    echo "   Puede ser por permisos o propagación DNS"
fi

# 9. Limpiar archivos temporales
rm -f "$VERSIONED_FIRMWARE" version.json
echo "🧹 Limpieza completada"

echo ""
echo "🎉 Despliegue completado!"
echo "📋 Resumen:"
echo "   - Versión: $VERSION"
echo "   - URL: $SERVER_URL/$VERSIONED_FIRMWARE"
echo "   - JSON: $SERVER_URL/version.json"
echo "   - Método: $TRANSFER_METHOD"
echo "   - Servidor: $SERVER_HOST"
echo "   - Path: $REMOTE_PATH"
echo ""
echo "⏰ Los dispositivos comenzarán a actualizarse en los próximos 5 minutos"