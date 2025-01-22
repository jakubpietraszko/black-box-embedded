# Flash
```
# add nimble library
# delete OTA

$HOME/esp/v5.3.1/esp-idf/install.sh         
sudo chmod 666 /dev/ttyUSB0
source $HOME/esp/v5.3.1/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```
# Erase flash
```
idf.py erase-flash
```

# MQTT server
```
sudo mosquitto -c /etc/mosquitto/mosquitto.conf -v
```

# Repair mosquitto
```
sudo lsof -i :1883
sudo kill <PID>

```

# Fronted
```
cd fronted/app
source .venv/bin/activate
python3.12 app.py
firefox http://127.0.0.1:5000
```

# Backend

```
cd fronted/json-server
node server.js
```
























# Flash
```
$HOME/esp/v5.3.1/esp-idf/install.sh         
sudo chmod 666 /dev/ttyUSB0
source $HOME/esp/v5.3.1/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```
idf.py erase-flash
# MQTT 
## Wlaczenie servera
```
sudo mosquitto -c /etc/mosquitto/mosquitto.conf -v
```

## Czytanie wszystkich danych
```
mosquitto_sub -h 192.168.145.179 -t "blackbox/#" -u "user" -P "password"
```

## Czytanie z jednego czujnika
```
mosquitto_sub -h 192.168.145.179 -t "blackbox/a0b765201334/#" -u "user" -P "password"
```
## Wizualizacja
```
cd mqtt_plots
source .venv/bin/activate
python main.py
```












$HOME/esp/v5.3.1/esp-idf/install.sh         
source $HOME/esp/v5.3.1/esp-idf/export.sh
idf.py build

```
wlaczenie servera
sudo mosquitto -c /etc/mosquitto/mosquitto.conf -v

polaczenie sie z serwerem i czytanie danych z tematu esp32/sensor/random
mosquitto_sub -h 192.168.145.179 -t "esp32/sensor/random" -u "user" -P "password"

```


```
klikamy guzik build 
wybieramy odpowiednie com np 3 
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

```
curl -X POST http://192.168.4.1/change_password -d "new_passwd12345"
```
