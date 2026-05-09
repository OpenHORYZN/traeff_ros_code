import ompl
print(dir(ompl))
print(ompl.__version__ if hasattr(ompl, '__version__') else "no version attr")

from ompl import base as ob
s = ob.RealVectorStateSpace(2)
bounds = ob.RealVectorBounds(2)
bounds.setLow(0); bounds.setHigh(1)
s.setBounds(bounds)
st = s.allocState()
print(type(st))
print(dir(st))

# Try these one by one:
try: print("getX:", st.getX())
except Exception as e: print("getX failed:", e)
try: print("values:", st.values)
except Exception as e: print("values failed:", e)
try: print("index:", st[0])
except Exception as e: print("index failed:", e)
