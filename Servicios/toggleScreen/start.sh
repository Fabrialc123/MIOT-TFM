# sudo chmod +x start.sh
# sudo mkdir /opt/toggleScreen
# sudo cp start.sh /opt/toggleScreen/start.sh
#!/bin/bash

DEVICE="/dev/input/event0"  # <-- cambia esto al que corresponda a tu botón
KEY_CODE="142"

# Estado actual de la pantalla
SCREEN_ON=true

# Funciones para controlar la pantalla
turn_off_display() {
    xset dpms force off
    SCREEN_ON=false
}

turn_on_display() {
    xset dpms force on
    SCREEN_ON=true
}

# Escucha eventos del dispositivo
evtest "$DEVICE" | while read line; do
    if echo "$line" | grep -q "code $KEY_CODE (KEY_SLEEP), value 0"; then
	echo "SCRREN_ON:" {$SCREEN_ON}
        if $SCREEN_ON; then
            turn_off_display
        else
            turn_on_display
        fi
    fi
done
