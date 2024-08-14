//=====================
//HTML code for webpage
//=====================

const char webpageCont[] PROGMEM =
  R"=====(
<!DOCTYPE HTML>
<html>
<title>ESP32_W6100 AsyncSocketServer</title>

<!---------------------------CSS-------------------------->

<style>

    h1 {font-size: 40px; color: red; text-align: center}

</style>

<!--------------------------HTML-------------------------->

<body>

    <h1>ESP32_W6100 AsyncSocketServer</h1>
        
</body>

<!----------------------JavaScript------------------------>

<script>

var websoc = new WebSocket('ws://'+window.location.hostname+':80/ws');

websoc.onopen = function() 
{
  window.alert("Client Connected");  
};      

</script>
</html>
)=====";