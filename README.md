# points_inspector

An inspection tool to see field values of sensor_msgs/msg/PointCloud2 messages.

Example:
```
--- points ---
frame_id:laser_data_frame
stamp   :1670294980 379087360
size    :512 x 128
x              : datatype=FLOAT32 mean=3.894 first=-0.000 last=-0.000 median=0.000 min=-5.003 max=53.615
y              : datatype=FLOAT32 mean=0.245 first=0.000 last=-0.000 median=0.000 min=-98.566 max=138.273
z              : datatype=FLOAT32 mean=0.406 first=0.000 last=-0.000 median=-0.000 min=-2.003 max=15.989
intensity      : datatype=FLOAT32 mean=120.222 first=0.000 last=0.000 median=4.000 min=0.000 max=9207.000
t              : datatype=UINT32 mean=1352700063.0 first=2690838016 last=2690838016 median=49959936 min=0 max=2690838016
reflectivity   : datatype=UINT16 mean=5.4 first=0 last=0 median=0 min=0 max=184
ring           : datatype=UINT8 mean=63.5 first=0 last=127 median=64 min=0 max=127
ambient        : datatype=UINT16 mean=391.8 first=0 last=0 median=293 min=0 max=6605
range          : datatype=UINT32 mean=6368.6 first=0 last=0 median=0 min=0 max=138284
```