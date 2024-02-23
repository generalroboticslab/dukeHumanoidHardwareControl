# Command input 
```
sudo ./simple_test ifname control_mode target max_velocity gear_ratio
```
- ifname: the name of adapter
- control_mode: "csp","csv"
- target:\
         if mode is csp, traget means target position, unit is rad \
         if mode is csv,target means target velocity, unit is rad/s
- max_velocity: the maximum velocity of motor.unit is rad/s
- gear_ratio: the gear ratio of motor


example
```
sudo ./test/linux/simple_test/simple_test enp3s0 csv 6.28 1 20
```



