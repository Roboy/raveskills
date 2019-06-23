import ravestate as rs
import luigi.states as states

with rs.Module(name="Luigi") as mod:

    for name, val in states.__dict__.items():
            if type(val).__name__ in ("State", "Signal", "Property"):
                mod.add(val)
