import numpy as np

def redraw_fn(f, axes):
  amp = float(f) / 3000
  f0 = 3
  t = np.arange(0.0, 1.0, 0.001)
  s = amp * np.sin(2 * np.pi * f0 * t)
  if not redraw_fn.initialized:
    redraw_fn.l, = axes.plot(t, s, lw=2, color='red')
    redraw_fn.initialized = True
  else:
    redraw_fn.l.set_ydata(s)

redraw_fn.initialized = False

videofig(100, redraw_fn)