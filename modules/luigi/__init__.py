import ravestate as rs
import ravestate_nlp as nlp
import ravestate_interloc as interloc
import ravestate_rawio as rawio

cost_per_scoop = 1  # TODO move to external config file that also lists the available flavors and payment options

with rs.Module(name="Luigi"):

    # -------------------- properties -------------------- #

    prop_flavor = rs.Property(name="flavor", allow_read=True, allow_write=True, always_signal_changed=True)
    prop_scoops = rs.Property(name="scoops", allow_read=True, allow_write=True, always_signal_changed=True)
    prop_flavor_scoop_tuple_list = rs.Property(name="flavor_scoop_tuple_list", default_value=[], allow_write=True,
                                               allow_read=True, always_signal_changed=True)

    # -------------------- signals -------------------- #

    sig_start_order_question = rs.Signal("start_order_question")
    sig_finish_order_question = rs.Signal("finish_order_question")
    sig_finished_payment = rs.Signal("finished_payment")
    sig_yesno_detected = rs.Signal("yesno")

    # -------------------- states: detection -------------------- #

    @rs.state(
        read=(nlp.prop_tokens, nlp.prop_triples, nlp.prop_lemmas),
        write=prop_flavor)
    def detect_flavor(ctx: rs.ContextWrapper):
        ice_cream_order = False
        tokens = ctx[nlp.prop_tokens]
        triples = ctx[nlp.prop_triples]
        lemmas = ctx[nlp.prop_lemmas]
        if "chocolate" in tokens or "vanilla" in tokens:
            if len(tokens) == 1:
                # this case holds when customer orders ice creams using just the flavor name
                ice_cream_order = True
            elif triples[0].match_either_lemma(subj={"i"}) and \
                    triples[0].match_either_lemma(pred={"want", "like", "desire", "have", "decide", "get",
                                                        "choose", "wish", "prefer"}) and \
                    triples[0].match_either_lemma(obj={"chocolate", "vanilla"}) and \
                    "not" not in lemmas:
                # this case holds when customer orders ice cream using phrases like
                # "i would like to have chocolate please"
                # "can i get some vanilla?"
                # "i want the chocolate ice cream!"
                # "i will have vanilla"
                # "i have decided for chocolate"
                # it does not hold whenever the customer negates his expression, i.e.
                # "i don't like chocolate"
                # "i have decided not to have vanilla"
                ice_cream_order = True
            elif triples[0].match_either_lemma(pred={"chocolate", "vanilla"}) and \
                    "no" not in lemmas and "not" not in lemmas:
                # this case holds when the order is phrased in a simple way like
                # "chocolate please"
                # "vanilla it is"
                # in case of negation the order is not recognized
                # "no chocolate ice cream please"
                # "it is definitely not vanilla"
                ice_cream_order = True
            elif triples[0].match_either_lemma(subj={"chocolate", "vanilla"}) and \
                    "no" not in lemmas and "not" not in lemmas:
                # this case holds when the customer makes his order as follows:
                # "vanilla sounds delicious"
                # "chocolate would be good"
                # again negations are ignored
                ice_cream_order = True
        if ice_cream_order:
            # TODO this part needs to be changed when ordering multiple flavors at the same time (possibly with scoops)
            if "chocolate" in tokens:
                ctx[prop_flavor] = "chocolate"
            if "vanilla" in tokens:
                ctx[prop_flavor] = "vanilla"

    @rs.state(
        cond=nlp.prop_ner.changed(),
        read=nlp.prop_ner,
        write=prop_scoops)
    def detect_scoops(ctx: rs.ContextWrapper):
        detected_scoops = extract_scoops(ctx[nlp.prop_ner])
        if detected_scoops:
            ctx[prop_scoops] = detected_scoops

    @rs.state(
        read=nlp.prop_yesno,
        signal=sig_yesno_detected)
    def yesno(ctx: rs.ContextWrapper):
        yesno = ctx[nlp.prop_yesno]
        if yesno == "yes" or yesno == "no":
            return rs.Emit()


    # -------------------- states: conversation flow -------------------- #

    @rs.state(
        cond=interloc.prop_all.pushed().detached().min_age(2),
        write=rawio.prop_out,
        signal=sig_start_order_question,
        emit_detached=True)
    def prompt_order(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = "guess what! i'm selling ice cream. want some?"
        return rs.Emit()

    @rs.state(
        cond=sig_start_order_question.max_age(-1) & sig_yesno_detected,
        read=nlp.prop_yesno,
        write=rawio.prop_out)
    def analyse_ice_cream_suggestion_answer(ctx: rs.ContextWrapper):
        if ctx[nlp.prop_yesno] == "yes":
            ctx[rawio.prop_out] = "i knew it - everyone loves ice cream! well, what can I get you?"
            # TODO list flavors that we have?
        elif ctx[nlp.prop_yesno] == "no":
            ctx[rawio.prop_out] = "hmm okay... well, what else can I do for you then?"

    @rs.state(
       cond=prop_flavor.changed().detached() | prop_scoops.changed().detached(),
       signal=sig_finish_order_question,
       emit_detached=True,
       write=(rawio.prop_out, prop_flavor_scoop_tuple_list, prop_flavor, prop_scoops))
    def check_scoops_flavor_combined(ctx: rs.ContextWrapper):
        if prop_flavor.read() and prop_scoops.read():
            ctx[rawio.prop_out] = "{scoops} scoops of the {flavor} will be delicious! is that all?". \
                format(scoops=prop_scoops.read(), flavor=prop_flavor.read())
            ctx[prop_flavor_scoop_tuple_list] = prop_flavor_scoop_tuple_list.read() + [(prop_flavor.read(),
                                                                                        prop_scoops.read())]
            ctx[prop_flavor] = None
            ctx[prop_scoops] = None
            return rs.Emit()
        elif prop_flavor.read():
            ctx[rawio.prop_out] = "{flavor} is also one of my favorites! can you also tell me how many scoops " \
                                  "you want?".format(flavor=prop_flavor.read())
        elif prop_scoops.read():
            ctx[rawio.prop_out] = "it would be helpful if you also told me what flavor you want {scoops} scoops " \
                                  "of...".format(scoops=prop_scoops.read())

    @rs.state(
       cond=sig_finish_order_question.max_age(-1) & sig_yesno_detected,
       read=nlp.prop_yesno,
       write=rawio.prop_out,
       emit_detached=True,
       signal=sig_finished_payment)
    def analyse_payment_suggestion_answer(ctx: rs.ContextWrapper):
        if ctx[nlp.prop_yesno] == "yes":
            complete_order, complete_cost = get_complete_order_and_cost(prop_flavor_scoop_tuple_list.read())
            ctx[rawio.prop_out] = "alrighty, payment time then! you owe me {cost} roboy coins for the {order}.". \
                format(cost=complete_cost, order=complete_order)
            # TODO add different payment options once we have them
            return rs.Emit()  # TODO signal should only be emitted once payment is completed
        elif ctx[nlp.prop_yesno] == "no":
            ctx[rawio.prop_out] = "wow, you are quite hungry. then tell me what other ice cream you want!"

    @rs.state(
       cond=sig_finished_payment,
       write=(rawio.prop_out, prop_flavor_scoop_tuple_list, prop_flavor, prop_scoops))
    def after_payment(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = "thanks for buying ice cream - enjoy and come back anytime you like!"
        ctx[prop_flavor_scoop_tuple_list] = []
        ctx[prop_flavor] = None
        ctx[prop_scoops] = None

    @rs.state(
       cond=interloc.prop_all.popped())
    def customer_left(ctx: rs.ContextWrapper):
        ctx[prop_flavor_scoop_tuple_list] = []
        ctx[prop_flavor] = None
        ctx[prop_scoops] = None


# -------------------- functions outside module -------------------- #

def get_complete_order_and_cost(flavor_scoop_tuple_list):
    order = ""
    cost = 0
    if len(flavor_scoop_tuple_list) == 1:
        order = "{scoops} scoops of {flavor}".format(flavor=flavor_scoop_tuple_list[0][0],
                                                     scoops=flavor_scoop_tuple_list[0][1])
        cost = cost_per_scoop * flavor_scoop_tuple_list[0][1]
    else:
        order_length = len(flavor_scoop_tuple_list)
        for i in range(0, order_length - 1):
            order += "{scoops} scoops of {flavor}, ".format(flavor=flavor_scoop_tuple_list[i][0],
                                                            scoops=flavor_scoop_tuple_list[i][1])
            cost += cost_per_scoop * flavor_scoop_tuple_list[i][1]
        order = order[:len(order)-2] + " "
        order += "and {scoops} scoops of {flavor}".format(flavor=flavor_scoop_tuple_list[order_length-1][0],
                                                          scoops=flavor_scoop_tuple_list[order_length-1][1])
        cost += cost_per_scoop * flavor_scoop_tuple_list[order_length-1][1]
    return order, cost


def extract_scoops(prop_ner):
    for entity, NE in prop_ner:
        if NE == "CARDINAL" and entity.isdigit():
            return int(entity)
        elif NE == "CARDINAL" and isinstance(entity, str):
            # we assume that no one orders more than 9 scoops of a flavor
            word2num = {
                "one": 1,
                "two": 2,
                "three": 3,
                "four": 4,
                "five": 5,
                "six": 6,
                "seven": 7,
                "eight": 8,
                "nine": 9,
            }
            return word2num[entity]
