from get_me_some_ice_cream.detection_states import *
from get_me_some_ice_cream.conversation_flow_states import *

with rs.Module(name="Ice Cream Dialogue") as mod:

    # Add further states here using mod.add(<state_name>)
    mod.add(detect_flavor)
    mod.add(detect_scoops)
    mod.add(prompt_order)
    mod.add(check_scoops_flavor_combined)
