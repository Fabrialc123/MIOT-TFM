# sudo chmod +x start.sh
# sudo mkdir /opt/reTerminalStartup
# sudo cp start.sh /opt/reTerminalStartup/start.sh
#!/bin/bash

export DISPLAY=:0

# Wait until graphics are ready
sleep 5

# Rotate touchscreen
xrandr --output DSI-1 --rotate right

# Set auto screen blanking after 5 min of inactivity
xset dpms 300

# Launch Grafana with Davis Dashboard in kiosk mode
/usr/bin/chromium-browser \
  --kiosk "http://10.42.0.1:3000/d/de73tr39n33lsd/davis?orgId=1&from=now-2y&to=now&timezone=browser&kiosk" \
  --noerrdialogs \
  --disable-session-crashed-bubble \
  --disable-infobars \
  --check-for-update-interval=604800 \
  --disable-pinch \
  --disable-gpu
