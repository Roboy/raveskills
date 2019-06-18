from get_me_some_ice_cream.detection_states import *

with rs.Module(name="Ice Cream Dialogue") as mod:

    # Add your states here using mod.add(<state_name>)
    mod.add(flavor_signal)
    mod.add(payment_signal)
    mod.add(predicate_want_signal)
    mod.add(flavor_order_received)
    mod.add(payment_request_received)
    mod.add(flavor_question_received)
    mod.add(payment_question_received)
