import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio

from get_me_some_ice_cream.states.detection import prop_flavor, prop_scoops

prop_flavor_scoop_tuple_list = rs.Property(name="flavor_scoop_tuple_list", allow_write=True, always_signal_changed=True)


@rs.state(
    cond=interloc.prop_all.pushed(),
    write=rawio.prop_out)
def prompt_order(ctx: rs.ContextWrapper):
    ctx[rawio.prop_out] = "Hey, want some ice cream?"


@rs.state(
    cond=prop_flavor.changed() | prop_scoops.changed(),
    write=rawio.prop_out)
def check_scoops_flavor_combined(ctx: rs.ContextWrapper):
    # TODO fill property prop_flavor_scoop_tuple_list and possibly ask for missing information
    if prop_flavor.read() is not None and prop_scoops.read() is not None:
        ctx[rawio.prop_out] = "I heard you ordered {} scoops of the flavor {}".format(prop_scoops.read(),
                                                                                      prop_flavor.read())
    elif prop_flavor.read() is not None:
        ctx[rawio.prop_out] = "I heard you ordered the flavor {}. How many scoops do you want?".format(prop_flavor.read())
    elif prop_scoops.read() is not None:
        ctx[rawio.prop_out] = "I heard you ordered {} scoops. What flavor do you want?".format(prop_scoops.read())
    else:
        ctx[rawio.prop_out] = "Hmmm... this shouldn't happen..."
