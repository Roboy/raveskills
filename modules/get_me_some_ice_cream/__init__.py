import ravestate as rs
import get_me_some_ice_cream.states as states

with rs.Module(name="Luigi") as mod:

    for name, val in states.__dict__.items():
            if type(val).__name__ in ("State", "Signal", "Property", "function"):
                mod.add(val)
