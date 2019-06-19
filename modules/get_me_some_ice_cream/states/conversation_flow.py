import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_nlp as nlp

from get_me_some_ice_cream.states.detection import prop_flavor, prop_scoops
from get_me_some_ice_cream.utils import get_complete_order_and_cost

cost_per_scoop = 1  # TODO move to external config file that also lists the available flavors and payment options

sig_start_order_question = rs.Signal("start_order_question")
sig_finish_order_question = rs.Signal("finish_order_question")
sig_finished_payment = rs.Signal("finished payment")

prop_flavor_scoop_tuple_list = rs.Property(name="flavor_scoop_tuple_list", default_value=[], allow_write=True,
                                           allow_read=True, always_signal_changed=True)


@rs.state(
    cond=interloc.prop_all.pushed().detached().min_age(2),
    write=rawio.prop_out,
    signal=sig_start_order_question)
def prompt_order(ctx: rs.ContextWrapper):
    ctx[rawio.prop_out] = "guess what! i'm selling ice cream. want some?"
    return rs.Emit()


@rs.state(
    cond=prop_flavor.changed() | prop_scoops.changed(),
    signal=sig_finish_order_question,
    emit_detached=True,
    write=rawio.prop_out)
def check_scoops_flavor_combined(ctx: rs.ContextWrapper):
    if prop_flavor.read() is not None and prop_scoops.read() is not None:
        ctx[rawio.prop_out] = "{scoops} scoops of the {flavor} will be delicious! is that all?".\
            format(scoops=prop_scoops.read(), flavor=prop_flavor.read())
        prop_flavor_scoop_tuple_list.write(prop_flavor_scoop_tuple_list.read()
                                           + [(prop_flavor.read(), prop_scoops.read())])
        prop_flavor.write(None)
        prop_scoops.write(None)
    elif prop_flavor.read() is not None:
        ctx[rawio.prop_out] = "{flavor} is also one of my favorites! can you also tell me how many scoops you want?".\
            format(flavor=prop_flavor.read())
    elif prop_scoops.read() is not None:
        ctx[rawio.prop_out] = "it would be helpful if you also told me what flavor you want {scoops} scoops of...".\
            format(scoops=prop_scoops.read())
    return rs.Emit()


@rs.state(
    cond=sig_finish_order_question & nlp.prop_yesno.changed().max_age(10),
    read=nlp.prop_yesno,
    write=rawio.prop_out,
    emit_detached=True,
    signal=sig_finished_payment)
def analyse_payment_suggestion_answer(ctx: rs.ContextWrapper):
    if ctx[nlp.prop_yesno] == "yes":
        complete_order, complete_cost = get_complete_order_and_cost(prop_flavor_scoop_tuple_list.read(), cost_per_scoop)
        ctx[rawio.prop_out] = "alrighty, payment time then! you owe me {cost} roboy coins for the {order}.".\
            format(cost=complete_cost, order=complete_order)
        # TODO add different payment options once we have them
    elif ctx[nlp.prop_yesno] == "no":
        ctx[rawio.prop_out] = "wow, you are quite hungry. then tell me what other ice cream you want!"
    return rs.Emit()  # TODO signal should only be emitted once payment is completed


@rs.state(
    cond=sig_finished_payment,
    write=rawio.prop_out)
def after_payment(ctx: rs.ContextWrapper):
    ctx[rawio.prop_out] = "thanks for buying ice cream - enjoy and come back anytime you like!"
    prop_flavor_scoop_tuple_list.write(None)
    prop_flavor.write(None)
    prop_scoops.write(None)


@rs.state(
    cond=interloc.prop_all.popped())
def customer_left():
    prop_flavor_scoop_tuple_list.write(None)
    prop_flavor.write(None)
    prop_scoops.write(None)


@rs.state(
    cond=sig_start_order_question & nlp.prop_yesno.changed(),
    read=nlp.prop_yesno,
    write=rawio.prop_out)
def analyse_ice_cream_suggestion_answer(ctx: rs.ContextWrapper):
    if ctx[nlp.prop_yesno] == "yes":
        ctx[rawio.prop_out] = "i knew it - everyone loves ice cream! well, what can I get you?"
        # TODO list flavors that we have?
    elif ctx[nlp.prop_yesno] == "no":
        ctx[rawio.prop_out] = "hmm okay... well, what else can I do for you then?"