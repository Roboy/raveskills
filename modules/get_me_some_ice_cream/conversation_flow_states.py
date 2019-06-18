import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio

from get_me_some_ice_cream import prop_flavor, prop_scoops

prop_flavor_scoop_tuple_list = rs.Property(name="flavor_scoop_tuple_list")


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
    if prop_flavor is None:
        ctx[rawio.prop_out] = "I heard you ordered the flavor {}".format(prop_flavor)
    elif prop_scoops is None:
        ctx[rawio.prop_out] = "I heard you ordered {} scoops".format(prop_scoops)
    else:
        ctx[rawio.prop_out] = "Hmmm... this shouldn't happen..."
