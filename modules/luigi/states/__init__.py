# Tried this but didn't work either
#
# import luigi.states.detection as detection
# import luigi.states.conversation_flow as convoflow
# from luigi.luigi import mod
#
# items = detection.__dict__.items()
# items += convoflow.__dict__.items()
#
# for name, val in items:
#     if type(val).__name__ is "Property":
#         mod.add(val)
#     if type(val).__name__ in ("State", "Signal"):
#         mod.add(val)
