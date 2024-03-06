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


example
```
//sudo ./simple_test enp3s0 csv 0.1,0.2,0.4,0.8,0.1,0.2 1
// sudo ./simple_test enp3s0 csp 0,0,0,0,0,0 1
```



