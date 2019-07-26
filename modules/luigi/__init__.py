import ravestate as rs
import ravestate_nlp as nlp
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_idle as idle
import rospy
import actionlib
from roboy_cognition_msgs.msg import OrderIceCreamAction, OrderIceCreamGoal
from roboy_cognition_msgs.srv import Payment
from enum import IntEnum
from ravestate_verbaliser import verbaliser
from os.path import realpath, dirname, join
verbaliser.add_folder(join(dirname(realpath(__file__))+"/phrases"))

cost_per_scoop = 1  # TODO move to external config file that also lists the available flavors and payment options

FLAVORS = {"chocolate", "vanilla"}
FLAVOR_SYNONYMS = {"flavor", "kind"}
SCOOP_SYNONYMS = {"scoop", "ball", "servings"}
DESIRE_SYNONYMS = {"want", "like", "desire", "have", "decide", "get", "choose", "wish", "prefer"}
NEGATION_SYNONYMS = {"no", "not"}
ICE_CREAM_SYNONYMS = {"icecream", "ice", "cream", "gelato", "sorbet"}
PAYMENT_OPTION_SYNONYMS = {"paypal", "cash", "coins", "coin", "money"}
PAY_SYNONYMS = {"pay", "want pay", "pay will", "pay can", "pay could", "use will", "like pay"}


class PaymentOptions(IntEnum):
    COIN = 0
    PAYPAL = 1


with rs.Module(name="Luigi"):

    # ----------- ROS scooping action client ---------------------

    rospy.init_node('scooping_client_py') # i am not sure about this, maybe you already have a node somewhere
    client = actionlib.SimpleActionClient('scooping_as', OrderIceCreamAction)
    client.wait_for_server()

    # -------------------- properties -------------------- #

    prop_flavors = rs.Property(name="flavor", always_signal_changed=False, default_value=[], allow_read=True,
                               allow_write=True)
    prop_scoops = rs.Property(name="scoops", always_signal_changed=False, default_value=[], allow_read=True,
                              allow_write=True)
    prop_flavor_scoop_tuple_list = rs.Property(name="flavor_scoop_tuple_list", default_value=[], allow_write=True,
                                               allow_read=True, always_signal_changed=False)
    prop_suggested_ice_cream = rs.Property(name="suggested_ice_cream", always_signal_changed=False, default_value=False,
                                           allow_read=True, allow_write=True)
    prop_price = rs.Property(name="price", always_signal_changed=False, default_value=-1, allow_read=True,
                             allow_write=True)
    prop_payment_option = rs.Property(name="payment_method", always_signal_changed=False, default_value=-1,
                                      allow_read=True, allow_write=True)
    prop_payment_success = rs.Property(name="payment_success", always_signal_changed=False, default_value=False,
                                       allow_read=True, allow_write=True)

    # -------------------- signals -------------------- #

    sig_start_order_question = rs.Signal("start_order_question")
    sig_finish_order_question = rs.Signal("finish_order_question")
    sig_start_payment= rs.Signal("start_payment")
    sig_finished_payment = rs.Signal("finished_payment")
    sig_payment_incomplete = rs.Signal("payment_incomplete")
    sig_yesno_detected = rs.Signal("yesno")
    sig_changed_flavor_or_scoops = rs.Signal("changed_flavor_or_scoops")
    sig_changed_payment_option = rs.Signal("changed_payment_option")
    sig_has_arrived = rs.Signal("has_arrived")  # TODO ad team should send this once Roboy has arrived
    sig_ice_cream_desire = rs.Signal("ice_cream_desire")
    sig_suggested_ice_cream = rs.Signal("suggested_ice_cream")
    sig_insert_coins = rs.Signal("insert_coins")

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
        read=(nlp.prop_triples, nlp.prop_lemmas),
        signal=sig_ice_cream_desire)
    def detect_ice_cream_desire(ctx: rs.ContextWrapper):
        triples = ctx[nlp.prop_triples]
        lemmas = ctx[nlp.prop_lemmas]
        if triples[0].match_either_lemma(subj={"i"}) and \
                triples[0].match_either_lemma(pred=DESIRE_SYNONYMS) and \
                triples[0].match_either_lemma(obj=ICE_CREAM_SYNONYMS) and \
                not NEGATION_SYNONYMS & set(lemmas):
            # signal is emitted when customer expresses the wish for ice cream using phrases like
            # "i would like to have ice cream please"
            # "can i get some ice cream?"
            # "i want ice cream!"
            return rs.Emit()

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
        cond=nlp.prop_tokens.changed(),
        read=(nlp.prop_tokens, nlp.prop_triples, nlp.prop_lemmas),
        write=prop_payment_option,
        signal=sig_changed_payment_option,
        emit_detached=True)
    def detect_payment_option(ctx: rs.ContextWrapper):
        detected_payment_option = False
        tokens = ctx[nlp.prop_tokens]
        triples = ctx[nlp.prop_triples]
        lemmas = ctx[nlp.prop_lemmas]
        if triples[0].match_either_lemma(subj={"i"}) and \
           triples[0].match_either_lemma(pred=PAY_SYNONYMS) and \
           triples[0].match_either_lemma(obj=PAYMENT_OPTION_SYNONYMS) and \
           not NEGATION_SYNONYMS & set(lemmas):
            # this case holds when customer answers the payment question using phrases like
            # "i would like to pay with cash please"
            # "i pay with paypal"
            # "can i pay with cash?"
            # it does not hold whenever the customer negates his expression, i.e.
            # "i don't want to pay with cash"
            # "i won't pay using paypal"
            detected_payment_option = True
        elif triples[0].match_either_lemma(pred=PAYMENT_OPTION_SYNONYMS) and \
                not NEGATION_SYNONYMS & set(lemmas):
            # this case holds when the customer answers the payment question with
            # "paypal please"
            # "cash"
            detected_payment_option = True
        elif triples[0].match_either_lemma(pred={"with", "by", "in"}) and \
                triples[0].match_either_lemma(obj=PAYMENT_OPTION_SYNONYMS) and \
                not NEGATION_SYNONYMS & set(lemmas):
            # this case holds when the customer answers the payment question with
            # "paypal please"
            # "by cash"
            # "paypal"
            # "in coins"
            detected_payment_option = True
        elif triples[0].match_either_lemma(pred={"with", "by", "in"}) and \
                triples[0].match_either_lemma(obj={"please"}) and \
                not NEGATION_SYNONYMS & set(lemmas):
            # this case holds when the customer answers the payment question with
            # "with paypal please"
            detected_payment_option = True
        elif triples[0].match_either_lemma(subj={"me"}) and \
                triples[0].match_either_lemma(pred={"let"}) and \
                triples[0].match_either_lemma(obj=PAYMENT_OPTION_SYNONYMS) and \
                not NEGATION_SYNONYMS & set(lemmas):
            # this case holds when customer answers the payment question using phrases like
            # "let me pay with cash please"
            detected_payment_option = True
        if detected_payment_option:
            ctx[prop_payment_option] = PaymentOptions.PAYPAL if "paypal" in tokens else PaymentOptions.COIN
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
        signal=sig_suggested_ice_cream,
        emit_detached=True)
    def prompt_order(ctx: rs.ContextWrapper):
        has_already_asked = ctx[prop_suggested_ice_cream]
        if not has_already_asked:
            ctx[rawio.prop_out] = verbaliser.get_random_question('greet_general')
            ctx[prop_suggested_ice_cream] = True
            return rs.Emit()

    @rs.state(
        cond=sig_suggested_ice_cream.max_age(-1) & sig_yesno_detected | sig_changed_flavor_or_scoops,
        read=(nlp.prop_yesno, prop_flavors, prop_scoops),
        write=rawio.prop_out,
        signal=sig_start_order_question,
        emit_detached=True)
    def analyse_ice_cream_suggestion_answer(ctx: rs.ContextWrapper):
        if ctx[prop_scoops] or ctx[prop_flavors]:
            return rs.Resign()
        if ctx[nlp.prop_yesno] == "yes":
            ctx[rawio.prop_out] = verbaliser.get_random_successful_answer("greet_general")
            return rs.Emit()
        elif ctx[nlp.prop_yesno] == "no":
            ctx[rawio.prop_out] = verbaliser.get_random_failure_answer("greet_general")

    @rs.state(
        cond=sig_ice_cream_desire.max_age(-1),
        write=rawio.prop_out,
        signal=sig_start_order_question,
        emit_detached=True)
    def ice_cream_desire_will_be_fulfilled(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = "you are talking to the right person, i can get you some ice cream!"
        return rs.Emit()


    @rs.state(
        cond=sig_has_arrived,
        write=rawio.prop_out)
    def arrived_at_location(ctx: rs.ContextWrapper):
            ctx[rawio.prop_out] = "hey you, nice to see you in person. now it's ice cream time!"

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
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("legit_order").format(order=possibly_complete_order)
            return rs.Emit()
        elif len(prop_flavors.read()) > len(prop_scoops.read()):
            current_order = [(prop_flavors.read()[i], prop_scoops.read()[i]) for i in range(0, len(prop_scoops.read()))]
            add_orders_together(current_order, prop_flavor_scoop_tuple_list.read())
            ctx[prop_flavor_scoop_tuple_list] = current_order
            ctx[prop_flavors] = prop_flavors.read()[len(prop_scoops.read()):]
            ctx[prop_scoops] = []
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("need_scoop").format(flavor=prop_flavors.read()[0])
        elif len(prop_flavors.read()) < len(prop_scoops.read()):
            current_order = [(prop_flavors.read()[i], prop_scoops.read()[i]) for i in range(0, len(prop_flavors.read()))]
            add_orders_together(current_order, prop_flavor_scoop_tuple_list.read())
            ctx[prop_flavor_scoop_tuple_list] = current_order
            ctx[prop_scoops] = prop_scoops.read()[len(prop_flavors.read()):]
            ctx[prop_flavors] = []
            if prop_scoops.read()[0] == 1:
                ctx[rawio.prop_out] = verbaliser.get_random_phrase("need_flavor").format(scoop=prop_scoops.read()[0],
                                                                                         s="")
            else:
                ctx[rawio.prop_out] = verbaliser.get_random_phrase("need_flavor").format(scoop=prop_scoops.read()[0],
                                                                                         s="s")

    @rs.state(
       cond=sig_finish_order_question.max_age(-1) & sig_yesno_detected,
       read=(prop_flavor_scoop_tuple_list, nlp.prop_yesno),
       write=(rawio.prop_out, prop_price),
       signal=sig_start_payment,
       emit_detached=True)
    def analyse_finish_order_answer(ctx: rs.ContextWrapper):
        if ctx[nlp.prop_yesno] == "yes":
            flavor_scoop_tuple_list = ctx[prop_flavor_scoop_tuple_list]
            complete_order, complete_cost = get_complete_order_and_cost(flavor_scoop_tuple_list)
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("preparing_order").format(order=complete_order)
            ctx[prop_price] = complete_cost * 100   # price is in cents
            return rs.Emit()
        elif ctx[nlp.prop_yesno] == "no":
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("continue_order")

    @rs.state(
        cond=sig_start_payment,
        read=prop_flavor_scoop_tuple_list,
        write=rawio.prop_out)
    def ask_payment_method(ctx: rs.ContextWrapper):
        flavor_scoop_tuple_list = ctx[prop_flavor_scoop_tuple_list]
        complete_order, complete_cost = get_complete_order_and_cost(flavor_scoop_tuple_list)
        flavors = [x for x, _ in flavor_scoop_tuple_list]
        scoops = [y for _, y in flavor_scoop_tuple_list]
        goal = OrderIceCreamGoal()  # use the action client declared in the beginning of the module
        goal.flavors = flavors
        goal.scoops = scoops
        client.send_goal(goal, feedback_cb=scooping_feedback_cb)
        client.wait_for_result()
        result = client.get_result()
        if result.success:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("payment"). \
                format(cost=complete_cost, order=complete_order)
        else:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("unexpected")  # TODO stop convo? output result.error?

    @rs.state(
        cond=sig_changed_payment_option.detached() | sig_payment_incomplete,
        read=prop_payment_option,
        write=rawio.prop_out,
        signal=sig_insert_coins,
        emit_detached=True)
    def start_payment(ctx: rs.ContextWrapper):
        # TODO add verbalizer for both cases below
        payment_option = ctx[prop_payment_option]
        if payment_option == PaymentOptions.PAYPAL:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("paypal_option")
            time.sleep(3)
            return rs.Emit()
        elif payment_option == PaymentOptions.COIN:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("coin_option")
            time.sleep(3)
            return rs.Emit()

    @rs.state(
        cond=sig_insert_coins,
        read=(prop_payment_option, prop_price),
        write=(rawio.prop_out, prop_payment_success, prop_price),
        signal=sig_finished_payment,
        emit_detached=True)
    def payment_process(ctx: rs.ContextWrapper):
        # TODO add verbalizer for all cases below
        payment_option = ctx[prop_payment_option]
        price = ctx[prop_price]
        amount_paid, error_message = payment_communication(price, payment_option)
        if amount_paid == 0:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("no_payment")
        elif amount_paid < price:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("amount_left") \
                                  .format(amount_left_over=amount_in_euros_and_cents(price - amount_paid))
            ctx[prop_price] = price - amount_paid
        elif amount_paid == price:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("perfect_amount")
            ctx[prop_payment_success] = True
        elif amount_paid > price:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("better_amount")\
                .format(amount_too_much=amount_in_euros_and_cents(amount_paid - price))
            ctx[prop_payment_success] = True
        elif error_message:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("error_payment")
        return rs.Emit()

    @rs.state(
        cond=sig_finished_payment,
        read=prop_payment_success,
        write=(rawio.prop_out, prop_flavor_scoop_tuple_list, prop_flavors, prop_scoops, prop_suggested_ice_cream,
               prop_price, prop_payment_success, prop_payment_option),
        signal=sig_payment_incomplete,
        emit_detached=True)
    def after_payment(ctx: rs.ContextWrapper):
        payment_success = ctx[prop_payment_success]
        if payment_success:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("luigi_farewell")
            ctx[prop_flavor_scoop_tuple_list] = []
            ctx[prop_flavors] = []
            ctx[prop_scoops] = []
            ctx[prop_suggested_ice_cream] = False
            ctx[prop_price] = -1
            ctx[prop_payment_option] = -1
            ctx[prop_payment_success] = False
        else:
            return rs.Emit()

    @rs.state(
       cond=interloc.prop_all.popped(),
       write=(rawio.prop_out, prop_flavor_scoop_tuple_list, prop_flavors, prop_scoops, prop_suggested_ice_cream))
    def customer_left(ctx: rs.ContextWrapper):
        ctx[prop_flavor_scoop_tuple_list] = []
        ctx[prop_flavors] = []
        ctx[prop_scoops] = []
        ctx[prop_suggested_ice_cream] = False
        ctx[prop_price] = -1
        ctx[prop_payment_option] = -1
        ctx[prop_payment_success] = False


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
        if token in FLAVORS:
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


def amount_in_euros_and_cents(amount):
    euros = amount // 100
    cents = amount % 100
    euro_string = "euro" if euros == 1 else "euros"
    cent_string = "cent" if cents == 1 else "cents"
    if euros and cents:
        return "{euros} {euro_string} and {cents} {cent_string}".format(euros=euros, euro_string=euro_string,
                                                                        cents=cents, cent_string=cent_string)
    elif euros:
        return "{euros} {euro_string}".format(euros=euros, euro_string=euro_string)
    elif cents:
        return "{cents} {cent_string}".format(cents=cents, cent_string=cent_string)


def scooping_feedback_cb(feedback):
    # TODO do something with the feedback like writing it to a global value that can be checked by an abort-state
    print('Feedback:', list(feedback.finished_flavors))


def payment_communication(price, payment_option):
    rospy.wait_for_service('payment')
    try:
        payment = rospy.ServiceProxy('payment', Payment)
        response = payment(np.uint16(price), np.uint8(payment_option))
        return response.amount_paid, response.error_message
    except rospy.ROSInterruptException as e:
        print('Service call failed:', e)
    # If luigi module is run without ROS, comment everything from above (including imports) and uncomment this:
    # print("in payment communication - price: {} option: {}".format(price, payment_option))
    # time.sleep(4)
    # return 220, ""
