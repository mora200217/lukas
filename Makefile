TARGET = micro_ros_servos 
PLATFORM = esp32
PORT =/dev/cu.usbserial-130
MPORT =/dev/ttyUSB0
BAUD = 115200 


upload: 
	esptool --chip ${PLATFORM} --port ${PORT} --baud ${BAUD} write-flash -z 0x1000 servos_microros/.pio/build/your_env/firmware.bin

docker_compile: 
	sudo docker run -it --rm --privileged \
	  --device=${PORT}:${MPORT} \
	  microros/micro-ros-agent:humble \
	  serial --dev ${MPORT} -b ${BAUD}
