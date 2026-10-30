import ctypes as c,subprocess,sys,time
lib=c.CDLL('libX11.so.6');lib.XOpenDisplay.argtypes=[c.c_char_p];lib.XOpenDisplay.restype=c.c_void_p
for f,args in [('XMoveResizeWindow',[c.c_void_p,c.c_ulong,c.c_int,c.c_int,c.c_uint,c.c_uint]),('XRaiseWindow',[c.c_void_p,c.c_ulong]),('XSetInputFocus',[c.c_void_p,c.c_ulong,c.c_int,c.c_ulong]),('XFlush',[c.c_void_p]),('XCloseDisplay',[c.c_void_p])]:getattr(lib,f).argtypes=args
D=lib.XOpenDisplay(None);assert D
w=int(sys.argv[1],0);lib.XMoveResizeWindow(D,w,80,90,1800,1100);lib.XRaiseWindow(D,w);lib.XSetInputFocus(D,w,2,0);lib.XFlush(D);time.sleep(1)
subprocess.run(['import','-window',hex(w),sys.argv[2]],check=True);lib.XCloseDisplay(D)
