import ravestate as rs
import ravestate_nlp as nlp
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_idle as idle


cost_per_scoop = 1  # TODO move to external config file that also lists the available flavors and payment options

FLAVORS = {"chocolate", "vanilla"}
FLAVOR_SYNONYMS = {"flavor", "kind"}
SCOOP_SYNONYMS = {"scoop", "ball", "servings"}
DESIRE_SYNONYMS = {"want", "like", "desire", "have", "decide", "get", "choose", "wish", "prefer"}
NEGATION_SYNONYMS = {"no", "not"}


with rs.Module(name="Luigi"):

    # -------------------- properties -------------------- #

    prop_flavors = rs.Property(name="flavor", always_signal_changed=False, default_value=[], allow_read=True, allow_write=True)
    prop_scoops = rs.Property(name="scoops", always_signal_changed=False, default_value=[], allow_read=True, allow_write=True)
    prop_flavor_scoop_tuple_list = rs.Property(name="flavor_scoop_tuple_list", default_value=[], allow_write=True,
                                               allow_read=True, always_signal_changed=False)
    prop_suggested_ice_cream = rs.Property(name="suggested_ice_cream", always_signal_changed=False, default_value=False, allow_read=True, allow_write=True)

    # -------------------- signals -------------------- #

    sig_start_order_question = rs.Signal("start_order_question")
    sig_finish_order_question = rs.Signal("finish_order_question")
    sig_finished_payment = rs.Signal("finished_payment")
    sig_yesno_detected = rs.Signal("yesno")
    sig_changed_flavor_or_scoops = rs.Signal("changed_flavor_or_scoops")

    # -------------------- states: detection -------------------- #

    @rs.state(
        cond=nlp.sig_is_question,
        read=nlp.prop_tokens,
        write=rawio.prop_out)
    def detect_specific_flavor_question(ctx: rs.ContextWrapper):
        tokens = ctx[nlp.prop_tokens]
        if FLAVORS & set(tokens):
            # TODO deal with question
            ctx[rawio.prop_out] = "you asked a question about some flavor..."
        else:
            return rs.Resign()

    @rs.state(
        cond=nlp.sig_is_question,
        read=nlp.prop_lemmas,
        write=rawio.prop_out)
    def detect_flavor_question(ctx: rs.ContextWrapper):
        lemmas = ctx[nlp.prop_lemmas]
        if FLAVOR_SYNONYMS & set(lemmas):
            ctx[rawio.prop_out] = "i'm selling chocolate and vanilla today, both are pretty yummy... " \
                                  "gonna be hard to choose for you"

    @rs.state(
        cond=nlp.prop_tokens.changed(),
        read=(nlp.prop_tokens, nlp.prop_triples, nlp.prop_lemmas, nlp.prop_ner, prop_flavors, prop_scoops),
        write=(prop_flavors, prop_scoops),
        signal=sig_changed_flavor_or_scoops,
        emit_detached=True)
    def detect_flavors_and_scoops(ctx: rs.ContextWrapper):
        ice_cream_order = False
        tokens = ctx[nlp.prop_tokens]
        triples = ctx[nlp.prop_triples]
        lemmas = ctx[nlp.prop_lemmas]
        ner = ctx[nlp.prop_ner]
        if len(tokens) == 1 and FLAVORS & set(tokens):
            # this case holds when customer orders ice creams using just the flavor name
            ice_cream_order = True
        elif len(tokens) == 1 and extract_scoops(ner):
            # this case holds when the customer simply states the number of scoops
            ice_cream_order = True
        elif extract_scoops(ner) and triples[0].match_either_lemma(pred=SCOOP_SYNONYMS):
            # this case holds when the customer states the number of scoops using
            # "three scoops please"
            # "2 servings of chocolate"
            ice_cream_order = True
        elif extract_scoops(ner) and triples[0].match_either_lemma(pred={"each"}):
            # this case holds when the customer states the number of scoops as follows
            # "one each"
            ice_cream_order = True
        elif triples[0].match_either_lemma(subj={"i"}) and \
                triples[0].match_either_lemma(pred=DESIRE_SYNONYMS) and \
                triples[0].match_either_lemma(obj=FLAVORS) and \
                not NEGATION_SYNONYMS & set(lemmas):
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
        elif triples[0].match_either_lemma(pred=FLAVORS) and not NEGATION_SYNONYMS & set(lemmas):
            # this case holds when the order is phrased in a simple way like
            # "chocolate please"
            # "vanilla it is"
            # in case of negation the order is not recognized
            # "no chocolate ice cream please"
            # "it is definitely not vanilla"
            ice_cream_order = True
        elif triples[0].match_either_lemma(obj=FLAVORS) and \
                triples[0].match_either_lemma(pred=SCOOP_SYNONYMS) and \
                not NEGATION_SYNONYMS & set(lemmas):
            # this case holds for orders such as
            # "one scoop of chocolate"
            # "two servings of vanilla please"
            ice_cream_order = True
        elif triples[0].match_either_lemma(subj=FLAVORS) and not NEGATION_SYNONYMS & set(lemmas):
            # this case holds when the customer makes his order as follows:
            # "vanilla sounds delicious"
            # "chocolate would be good"
            # again negations are ignored
            ice_cream_order = True
        if ice_cream_order:
            flavors = extract_flavors(tokens)
            scoops = extract_scoops(ner)
            if "each" in tokens and len(scoops) == 1:
                scoops *= len(flavors)
            if flavors:
                ctx[prop_flavors] = prop_flavors.read() + flavors
            if scoops:
                ctx[prop_scoops] = prop_scoops.read() + scoops
            return rs.Emit()

    @rs.state(
        read=nlp.prop_yesno,
        signal=sig_yesno_detected)
    def yesno_detection(ctx: rs.ContextWrapper):
        yesno = ctx[nlp.prop_yesno]
        if yesno == "yes" or yesno == "no":
            return rs.Emit()


    # -------------------- states: conversation flow -------------------- #

    @rs.state(
        cond=interloc.prop_all.pushed().detached().min_age(2) | idle.sig_bored.min_age(1),
        read=prop_suggested_ice_cream,
        write=(rawio.prop_out, prop_suggested_ice_cream),
        signal=sig_start_order_question,
        emit_detached=True)
    def prompt_order(ctx: rs.ContextWrapper):
        has_already_asked = ctx[prop_suggested_ice_cream]
        if not has_already_asked:
            ctx[rawio.prop_out] = "guess what! i'm selling ice cream. want some?"
            ctx[prop_suggested_ice_cream] = True
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
        cond=sig_changed_flavor_or_scoops,
        signal=sig_finish_order_question,
        emit_detached=True,
        read=(prop_flavor_scoop_tuple_list, prop_flavors, prop_scoops),
        write=(rawio.prop_out, prop_flavor_scoop_tuple_list, prop_flavors, prop_scoops))
    def check_scoops_flavor_combined(ctx: rs.ContextWrapper):
        if prop_flavors.read() and prop_scoops.read() and len(prop_flavors.read()) == len(prop_scoops.read()):
            current_order = [x for x in zip(prop_flavors.read(), prop_scoops.read())]
            add_orders_together(current_order, prop_flavor_scoop_tuple_list.read())
            ctx[prop_flavor_scoop_tuple_list] = current_order
            ctx[prop_flavors] = []
            ctx[prop_scoops] = []
            possibly_complete_order, _ = get_complete_order_and_cost(prop_flavor_scoop_tuple_list.read())
            ctx[rawio.prop_out] = "{order} will be delicious! is that all?".format(order=possibly_complete_order)
            return rs.Emit()
        elif len(prop_flavors.read()) > len(prop_scoops.read()):
            current_order = [(prop_flavors.read()[i], prop_scoops.read()[i]) for i in range(0, len(prop_scoops.read()))]
            add_orders_together(current_order, prop_flavor_scoop_tuple_list.read())
            ctx[prop_flavor_scoop_tuple_list] = current_order
            ctx[prop_flavors] = prop_flavors.read()[len(prop_scoops.read()):]
            ctx[prop_scoops] = []
            ctx[rawio.prop_out] = "can you also tell me how many scoops " \
                                  "you want of {flavor}?".format(flavor=prop_flavors.read()[0])
        elif len(prop_flavors.read()) < len(prop_scoops.read()):
            current_order = [(prop_flavors.read()[i], prop_scoops.read()[i]) for i in range(0, len(prop_flavors.read()))]
            add_orders_together(current_order, prop_flavor_scoop_tuple_list.read())
            ctx[prop_flavor_scoop_tuple_list] = current_order
            ctx[prop_scoops] = prop_scoops.read()[len(prop_flavors.read()):]
            ctx[prop_flavors] = []
            if prop_scoops.read()[0] == 1:
                ctx[rawio.prop_out] = "it would be helpful if you also told me what flavor you want {scoop} scoop " \
                                      "of...".format(scoop=prop_scoops.read()[0])
            else:
                ctx[rawio.prop_out] = "it would be helpful if you also told me what flavor you want {scoops} scoops " \
                                  "of...".format(scoops=prop_scoops.read()[0])


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
       write=(rawio.prop_out, prop_flavor_scoop_tuple_list, prop_flavors, prop_scoops))
    def after_payment(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = "thanks for buying ice cream - enjoy and come back anytime you like!"
        ctx[prop_flavor_scoop_tuple_list] = []
        ctx[prop_flavors] = []
        ctx[prop_scoops] = []

    @rs.state(
       cond=interloc.prop_all.popped())
    def customer_left(ctx: rs.ContextWrapper):
        ctx[prop_flavor_scoop_tuple_list] = []
        ctx[prop_flavors] = []
        ctx[prop_scoops] = []


# -------------------- functions outside module -------------------- #


def get_complete_order_and_cost(flavor_scoop_tuple_list):
    order = ""
    cost = 0
    if len(flavor_scoop_tuple_list) == 1:
        if flavor_scoop_tuple_list[0][1] == 1:
            order += "{scoops} scoop of {flavor}".format(flavor=flavor_scoop_tuple_list[0][0],
                                                           scoops=flavor_scoop_tuple_list[0][1])
        else:
            order += "{scoops} scoops of {flavor}".format(flavor=flavor_scoop_tuple_list[0][0],
                                                            scoops=flavor_scoop_tuple_list[0][1])
        cost = cost_per_scoop * flavor_scoop_tuple_list[0][1]
    else:
        order_length = len(flavor_scoop_tuple_list)
        for i in range(0, order_length - 1):
            if flavor_scoop_tuple_list[i][1] == 1:
                order += "{scoops} scoop of {flavor}, ".format(flavor=flavor_scoop_tuple_list[i][0],
                                                                scoops=flavor_scoop_tuple_list[i][1])
            else:
                order += "{scoops} scoops of {flavor}, ".format(flavor=flavor_scoop_tuple_list[i][0],
                                                                scoops=flavor_scoop_tuple_list[i][1])
            cost += cost_per_scoop * flavor_scoop_tuple_list[i][1]
        order = order[:len(order)-2]
        if flavor_scoop_tuple_list[order_length-1][1] == 1:
            order += " and {scoops} scoop of {flavor}".format(flavor=flavor_scoop_tuple_list[order_length-1][0],
                                                           scoops=flavor_scoop_tuple_list[order_length-1][1])
        else:
            order += " and {scoops} scoops of {flavor}".format(flavor=flavor_scoop_tuple_list[order_length-1][0],
                                                            scoops=flavor_scoop_tuple_list[order_length-1][1])
        cost += cost_per_scoop * flavor_scoop_tuple_list[order_length-1][1]
    return order, cost


def extract_flavors(prop_tokens):
    flavors = []
    for token in prop_tokens:
        if token in ("vanilla", "chocolate"):
            flavors += [token]
    return flavors


def extract_scoops(prop_ner):
    scoops = []
    for entity, NE in prop_ner:
        if NE == "CARDINAL" and entity.isdigit():
            scoops += [int(entity)]
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
            scoops += [word2num[entity]]
    return scoops


def list_contains_flavor(tuple_list, flavor):
    for f, amount in tuple_list:
        if f == flavor:
            return amount
    return None


def add_orders_together(current_order, old_order):
    for flavor, amount in old_order:
        curr_amount = list_contains_flavor(current_order, flavor)
        if curr_amount:
            total_amount = amount + curr_amount
            current_order.remove((flavor, curr_amount))
            current_order.append((flavor, total_amount))
        else:
            current_order.append((flavor, amount))
