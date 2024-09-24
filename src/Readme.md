# Command input 
```
sudo ./simple_test ifname control_mode target max_velocity
```
- ifname: the name of adapter
- control_mode: "csp","csv"
- target:\
         if mode is csp, traget means target position, unit is rad \
         if mode is csv,target means target velocity, unit is rad/s
- max_velocity: the maximum velocity of motor.unit is rad/s, a comma separated string


example (e.g enp3s0 is the interface_name, change "enp3s0" to your adaptor name for the following code)
```
//sudo ./simple_test enp3s0 csv 0,0,0,0,0,0 1 40
// sudo ./simple_test enp3s0 csp 0,0,0,0,0,0 1 40

sudo ./ext/SOEM/test/linux/slaveinfo/slaveinfo enp3s0 -sdo


// restart interface
sudo ip link set enp3s0 down
sudo ip link set enp3s0 up
```



