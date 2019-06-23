import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio

from luigi.states.detection import prop_flavor, prop_scoops

prop_flavor_scoop_tuple_list = rs.Property(name="flavor_scoop_tuple_list", default_value=[], allow_write=True,
                                           allow_read=True, always_signal_changed=True)


@rs.state(
    cond=interloc.prop_all.pushed().detached().min_age(2),
    write=rawio.prop_out)
def prompt_order(ctx: rs.ContextWrapper):
    ctx[rawio.prop_out] = "Want some ice cream?"


@rs.state(
    cond=prop_flavor.changed() | prop_scoops.changed(),
    write=rawio.prop_out)
def check_scoops_flavor_combined(ctx: rs.ContextWrapper):
    if prop_flavor.read() and prop_scoops.read():
        ctx[rawio.prop_out] = "I heard you ordered {scoops} scoops of the flavor {flavor}".\
            format(scoops=prop_scoops.read(), flavor=prop_flavor.read())
        prop_flavor_scoop_tuple_list.write(prop_flavor_scoop_tuple_list.read() + [(prop_flavor.read(), prop_scoops.read())])
        prop_flavor.write(None)
        prop_scoops.write(None)
    elif prop_flavor.read():
        ctx[rawio.prop_out] = "I heard you ordered the flavor {flavor}. How many scoops do you want?".\
            format(flavor=prop_flavor.read())
    elif prop_scoops.read():
        ctx[rawio.prop_out] = "I heard you ordered {scoops} scoops. What flavor do you want?".\
            format(scoops=prop_scoops.read())
    else:
        ctx[rawio.prop_out] = "Hmmm... this shouldn't happen..."
