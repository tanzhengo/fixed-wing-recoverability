import ctypes as c,time,math,subprocess,json
from pathlib import Path
x=c.CDLL('libX11.so.6');x.XOpenDisplay.restype=c.c_void_p;x.XOpenDisplay.argtypes=[c.c_char_p]
x.XRaiseWindow.argtypes=[c.c_void_p,c.c_ulong];x.XSetInputFocus.argtypes=[c.c_void_p,c.c_ulong,c.c_int,c.c_ulong];x.XFlush.argtypes=[c.c_void_p];x.XKeysymToKeycode.argtypes=[c.c_void_p,c.c_ulong];x.XKeysymToKeycode.restype=c.c_uint
xt=c.CDLL('libXtst.so.6');xt.XTestFakeKeyEvent.argtypes=[c.c_void_p,c.c_uint,c.c_int,c.c_ulong]
d=x.XOpenDisplay(None);w=0x6e00016;x.XRaiseWindow(d,w);x.XSetInputFocus(d,w,2,0);x.XFlush(d);time.sleep(.2)
k=x.XKeysymToKeycode(d,0xff1b);xt.XTestFakeKeyEvent(d,k,1,0);xt.XTestFakeKeyEvent(d,k,0,0);x.XFlush(d);time.sleep(.2)
pitch=.46;yaw=2.22
q=(-math.sin(pitch/2)*math.sin(yaw/2),math.sin(pitch/2)*math.cos(yaw/2),math.cos(pitch/2)*math.sin(yaw/2),math.cos(pitch/2)*math.cos(yaw/2))
msg='camera {name: "user_camera" view_controller: "orbit" pose {position {x: 480 y: -680 z: 430} orientation {x: %s y: %s z: %s w: %s}}}'%q
Path('work/interface_screenshots/gazebo_camera_message.txt').write_text(msg+'\n')
subprocess.run(['gz','topic','-p','/gazebo/default/gui','-m',msg],check=True)
