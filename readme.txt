loacte '99-usb-serial.rules' :
/etc/udev/rules.d

use this code
$sudo cp 99-usb-serial.rules /etc/udev/rules.d/

#port rule reload. after this code, reconnect usb
$sudo udevadm control --reload-rules

#you can check rule changed by
$ls /dev/OpenRB150*
