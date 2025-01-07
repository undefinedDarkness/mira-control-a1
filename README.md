# Mira Control Mavlink Node
Our original implementation can be seen in [original.py](https://github.com/undefinedDarkness/mira-control-a1/blob/main/original.py)

This is a re-implemented mavlink ROS node to communicate with a PixHawk for the operation of our AUV (Mira), The old pymavlink node has be entirely rewritten in Golang in order to improve asynchronous performance
and hopefully allow us to miss fewer telemetry messages from the PixHawk

The reason for the re-write was mainly to move away from the extremely asynchronous nature of our previous implementation and to reduce load on the CPU as the previous implementation was consuming a good chunk of the CPU of our raspberry Pi

## Architecture
![image](https://github.com/user-attachments/assets/e4dc372e-b92d-4a27-974f-8fb3b3c09561)

## TODO
- [ ] Resolve issue where commands are not accepted by the PixHawk
