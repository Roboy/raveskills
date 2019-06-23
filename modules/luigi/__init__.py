from luigi.states.detection import *
from luigi.states.conversation_flow import *

with rs.Module(name="Luigi") as mod:

    # Add further states here using mod.add(<state_name>)
    mod.add(prop_scoops)
    mod.add(prop_flavor)
    mod.add(prop_flavor_scoop_tuple_list)
    mod.add(detect_flavor)
    mod.add(detect_scoops)
    mod.add(prompt_order)
    mod.add(check_scoops_flavor_combined)
