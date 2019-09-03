import ravestate as rs
import ravestate_nlp as nlp
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_idle as idle
import ravestate_phrases_basic_en as lang
import numpy as np
import time
from os.path import realpath, dirname, join
from ravestate_verbaliser import verbaliser, prop_intent
from enum import IntEnum
from reggol import get_logger

logger = get_logger(__name__)
verbaliser.add_folder(join(dirname(realpath(__file__)) + "/phrases"))

ROS_AVAILABLE = True
try:
    import rospy
    import actionlib
    import ravestate_ros1 as ros1
    from roboy_cognition_msgs.msg import OrderIceCreamAction, OrderIceCreamGoal
    from roboy_cognition_msgs.srv import Payment
    from std_msgs.msg import Bool
except ImportError:
    logger.error("Could not import ROS1 dependencies. Please make sure to have ROS1 installed.")
    ROS_AVAILABLE = False

DESCRIPTOR_SYNONYMS = {"good", "amazing","happy"}
FLAVORS = {"chocolate", "choco", "vanilla", "strawberry", "chocolates", "strawberries", "vanillas"}
FLAVOR_SYNONYMS = {"flavor", "kind"}
SCOOP_SYNONYMS = {"scoop", "ball", "servings","scoops","balls"}
IMPERATIVE_SYNOYNMS = {"give", "make", "serve"}
DESIRE_SYNONYMS = {"want", "like", "desire", "have", "decide", "get", "choose", "wish", "prefer"}
POLITE_SYNONYMS = {"please", "bitte"}
NEGATION_SYNONYMS = {"no", "not"}
ICE_CREAM_SYNONYMS = {"icecream", "ice", "cream", "gelato", "sorbet"}
PAYMENT_OPTION_SYNONYMS = {"paypal", "cash", "coins", "coin", "money"}
PAY_SYNONYMS = {"pay", "want pay", "pay will", "pay can", "pay could", "use will", "like pay"}

cost_per_scoop = 1  # TODO move to external config file that also lists the available flavors and payment options


class PaymentOptions(IntEnum):
    COIN = 0
    PAYPAL = 1


class ScoopingFeedbackMessages(IntEnum):
    NONE = -1
    TIME = -2


class DetectionStates(IntEnum):
    NOT_SET = -1
    OUT = 0
    IN = 1


with rs.Module(name="Luigi"):

    # -------------------- properties -------------------- #

    prop_flavors = rs.Property(name="flavor", always_signal_changed=False, default_value=[], allow_read=True,
                               allow_write=True)
    prop_order_verified = rs.Property(name="order_verified", always_signal_changed=False, default_value=False, allow_read=True,
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
    prop_asked_order_count = rs.Property(name="asked_order_count", always_signal_changed=False, default_value=1,
                                         allow_read=True, allow_write=True)
    prop_asked_payment_count = rs.Property(name="asked_payment_count", always_signal_changed=False, default_value=1,
                                           allow_read=True, allow_write=True)
    prop_ice_cream_desire_detection = rs.Property(name="ice_cream_desire_detection", always_signal_changed=False,
                                                  default_value=DetectionStates.NOT_SET,
                                                  allow_read=True,
                                                  allow_write=True)
    prop_flavors_and_scoops_detection = rs.Property(name="flavors_and_scoops_detection", always_signal_changed=False,
                                                    default_value=DetectionStates.NOT_SET,
                                                    allow_read=True,
                                                    allow_write=True)
    prop_payment_option_detection = rs.Property(name="payment_option_detection", always_signal_changed=False,
                                                default_value=DetectionStates.NOT_SET,
                                                allow_read=True,
                                                allow_write=True)
    prop_yesno_detection = rs.Property(name="yesno_detection", always_signal_changed=False,
                                       default_value=DetectionStates.NOT_SET,
                                       allow_read=True,
                                       allow_write=True)

    # -------------------- signals -------------------- #

    sig_finish_order_question = rs.Signal("finish_order_question")
    sig_start_payment = rs.Signal("start_payment")
    sig_finished_payment = rs.Signal("finished_payment")
    sig_payment_incomplete = rs.Signal("payment_incomplete")
    sig_yesno_detected = rs.Signal("yesno")
    sig_changed_flavor_or_scoops = rs.Signal("changed_flavor_or_scoops")
    sig_changed_payment_option = rs.Signal("changed_payment_option")
    sig_has_arrived = rs.Signal("has_arrived")  # TODO ad team should send this once Roboy has arrived
    sig_ice_cream_desire = rs.Signal("ice_cream_desire")
    sig_suggested_ice_cream = rs.Signal("suggested_ice_cream")
    sig_insert_coins_or_scan_qr = rs.Signal("insert_coins_or_scan_qr")
    sig_ask_again_order = rs.Signal("ask_again_order")
    sig_asked_payment_method = rs.Signal("asked_payment_method")
    sig_send_to_scooping = rs.Signal("send_to_scooping")
    sig_loop_feedback = rs.Signal("loop_feedback")
    sig_wait_for_cup = rs.Signal("wait_for_cup")
    sig_wait_for_telegram_customer_to_come_close = rs.Signal("wait_for_telegram_customer_to_come_close")

    # ----------- ROS client and methods ---------------- #

    if ROS_AVAILABLE:
        client = actionlib.SimpleActionClient('luigi_scoop', OrderIceCreamAction)

        has_arrived = ros1.Ros1SubProperty(
            name="has_arrived",
            topic="/roboy/autonomousdriving/arrival",
            msg_type=Bool,
            always_signal_changed=True)

        @rs.state(
            cond=has_arrived.changed().detached(),
            read=has_arrived,
            write=rawio.prop_out,
            signal=sig_wait_for_telegram_customer_to_come_close,
            emit_detached=True)
        def arrived_at_location(ctx: rs.ContextWrapper):
            if ctx[has_arrived]:
                ctx[rawio.prop_out] = verbaliser.get_random_phrase("telegram_arrival")
                return rs.Emit(wipe=True)

    class ScoopingCommunication:
        def __init__(self):
            self.feedback = None
            self.last_scooping_feedback_array = None
            self.stop_feedback = False

        def send_order(self, flavor_scoop_tuple_list):
            self.feedback = None
            self.stop_feedback = False
            self.last_scooping_feedback_array = None
            if ROS_AVAILABLE:
                flavors = [x for x, _ in flavor_scoop_tuple_list]
                scoops = [y for _, y in flavor_scoop_tuple_list]
                goal = OrderIceCreamGoal()
                goal.flavors = flavors
                goal.scoops = scoops
                client.send_goal(goal, feedback_cb=self.scooping_feedback_cb)
                client.wait_for_result()
            else:
                self.stop_feedback = True

        def scooping_feedback_cb(self, feedback):
            self.feedback = (list(feedback.finished_scoops), feedback.status_message)

    scooping_communication = ScoopingCommunication()

    def payment_communication(price, payment_option, flavor_scoop_tuple_list):
        flavors = [x for x, _ in flavor_scoop_tuple_list]
        scoops = [y for _, y in flavor_scoop_tuple_list]
        if ROS_AVAILABLE:
            rospy.wait_for_service('payment')
            try:
                payment = rospy.ServiceProxy('payment', Payment)
                response = payment(np.uint16(price), np.uint8(payment_option), flavors, scoops)
                return response.amount_paid, response.error_message  # TODO response.customer_name can be used for stuff
            except rospy.ROSInterruptException as e:
                logger.error('Service call failed:', e)
        else:
            return 242, "", ""


    # -------------------- states: detection -------------------- #

    @rs.state(
        cond=nlp.prop_tokens.changed(),
        read=(nlp.prop_triples, nlp.prop_lemmas),
        write=prop_ice_cream_desire_detection,
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
            ctx[prop_ice_cream_desire_detection] = DetectionStates.IN
            return rs.Emit(wipe=True)
        ctx[prop_ice_cream_desire_detection] = DetectionStates.OUT

    @rs.state(
        cond=nlp.prop_tokens.changed(),
        read=(nlp.prop_tokens, nlp.prop_triples, nlp.prop_lemmas, nlp.prop_ner, prop_flavors, prop_scoops),
        write=(prop_scoops, prop_flavors, prop_asked_order_count, prop_flavors_and_scoops_detection),
        signal=sig_changed_flavor_or_scoops,
        emit_detached=True)
    def detect_flavors_and_scoops(ctx: rs.ContextWrapper):
        logger.info("Entered detect flavors again with")
        ctx[prop_asked_order_count] = 1
        ice_cream_order = False
        tokens = ctx[nlp.prop_tokens]
        triples = ctx[nlp.prop_triples]
        lemmas = ctx[nlp.prop_lemmas]
        ner = ctx[nlp.prop_ner]
        logger.warn(ner)
        if len(tokens) == 1 and FLAVORS & set(tokens):
            # this case holds when customer orders ice creams using just the flavor name
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
        # elif triples[0].match_either_lemma(subj={"i"}) and \
        #         triples[0].match_either_lemma(pred=DESIRE_SYNONYMS) and \
        #         triples[0].match_either_lemma(obj=FLAVORS) and \
        #         not NEGATION_SYNONYMS & set(lemmas):

        elif triples[0].match_either_lemma(pred=DESIRE_SYNONYMS) and \
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
        elif extract_scoops(ner) and not NEGATION_SYNONYMS & set(lemmas):
            # this case holds when the order is phrased in a simple way like
            # "two please"
            # "three"
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
        elif DESIRE_SYNONYMS & set(tokens) and FLAVORS & set(tokens) and not NEGATION_SYNONYMS  & set(lemmas):
            # I think I want to have two scoops of vanilla
            ice_cream_order = True
        elif DESCRIPTOR_SYNONYMS & set(tokens) and not NEGATION_SYNONYMS  & set(lemmas) and (FLAVORS & set(tokens) or SCOOP_SYNONYMS & set(tokens)):
            # i think two scoops of vanilla sounds good
            # what an amazing day to have 3 scoops of ice cream
            # 2 vanilla sounds good
            ice_cream_order = True
        elif IMPERATIVE_SYNOYNMS & set(tokens) and not NEGATION_SYNONYMS  & set(lemmas) and (FLAVORS & set(tokens) or SCOOP_SYNONYMS & set(tokens)):
            # give me three
            # serve me chocolate
            # can you give me four vanilla
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
            ctx[prop_flavors_and_scoops_detection] = DetectionStates.IN
            return rs.Emit(wipe=True)
        ctx[prop_flavors_and_scoops_detection] = DetectionStates.OUT

    @rs.state(
        cond=nlp.prop_tokens.changed(),
        read=(nlp.prop_tokens, nlp.prop_triples, nlp.prop_lemmas),
        write=(prop_payment_option, prop_payment_option_detection),
        signal=sig_changed_payment_option,
        emit_detached=True)
    def detect_payment_option(ctx: rs.ContextWrapper):
        detected_payment_option = False
        tokens = ctx[nlp.prop_tokens]
        triples = ctx[nlp.prop_triples]
        lemmas = ctx[nlp.prop_lemmas]
        logger.info("Entering payment detection")
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
            # "in coins"
            detected_payment_option = True
        elif triples[0].match_either_lemma(pred={"with", "by", "in"}) and \
                triples[0].match_either_lemma(obj={"please"}) and \
                not NEGATION_SYNONYMS & set(lemmas):
            # this case holds when the customer answers the payment question with
            # "with paypal please"
            detected_payment_option = True
        elif triples[0].match_either_lemma(pred={"let"}) and \
                triples[0].match_either_lemma(obj=PAYMENT_OPTION_SYNONYMS) and \
                not NEGATION_SYNONYMS & set(lemmas):
            # this case holds when customer answers the payment question using phrases like
            # "let me pay with cash please"
            detected_payment_option = True

        if detected_payment_option:
            logger.info("Exiting payment detection")
            ctx[prop_payment_option] = PaymentOptions.PAYPAL if "paypal" in tokens else PaymentOptions.COIN
            ctx[prop_payment_option_detection] = DetectionStates.IN
            return rs.Emit(wipe=True)
        ctx[prop_payment_option_detection] = DetectionStates.OUT

    @rs.state(
        read=nlp.prop_yesno,
        write=prop_yesno_detection,
        signal=sig_yesno_detected)
    def yesno_detection(ctx: rs.ContextWrapper):
        if ctx[nlp.prop_yesno].yes() or ctx[nlp.prop_yesno].no():
            ctx[prop_yesno_detection] = True
            ctx[prop_yesno_detection] = DetectionStates.IN
            return rs.Emit(wipe=True)
        ctx[prop_yesno_detection] = DetectionStates.OUT

    @rs.state(
        cond=(
            prop_yesno_detection.changed() and prop_payment_option_detection.changed() and
            prop_ice_cream_desire_detection.changed() and prop_flavors_and_scoops_detection.changed()
        ),
        read=(
            prop_yesno_detection, prop_payment_option_detection, prop_ice_cream_desire_detection,
            prop_flavors_and_scoops_detection, rawio.prop_out
        ),
        write=(
            prop_yesno_detection, prop_payment_option_detection, prop_ice_cream_desire_detection,
            prop_flavors_and_scoops_detection, rawio.prop_out
        ))
    def detections_understood(ctx: rs.ContextWrapper):
        detections = [
            ctx[prop_yesno_detection],
            ctx[prop_payment_option_detection],
            ctx[prop_ice_cream_desire_detection],
            ctx[prop_flavors_and_scoops_detection]
        ]

        ctx[prop_yesno_detection] = DetectionStates.NOT_SET
        ctx[prop_payment_option_detection] = DetectionStates.NOT_SET
        ctx[prop_ice_cream_desire_detection] = DetectionStates.NOT_SET
        ctx[prop_flavors_and_scoops_detection] = DetectionStates.NOT_SET

        for detection in detections:
            if detection == DetectionStates.IN:
                return

        if not (ctx[rawio.prop_out] in verbaliser.get_phrase_list(lang.intent_greeting)):
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("listening_error")

    @rs.state(cond=rs.sig_startup)
    def start_state(ctx: rs.ContextWrapper):
        rospy.set_param('roboy_is_busy', False)
        print("START_STATE:" + str(rospy.get_param('roboy_is_busy')))

    @rs.state(
        cond=interloc.prop_all.pushed() | interloc.prop_all.popped(),
        read=interloc.prop_all)
    def is_busy(ctx: rs.ContextWrapper):
        busy = True if any(ctx.enum(interloc.prop_all)) else False

        # TODO Set this as param if you want to use inside ws_comm, o.w you can just assign a variable
        rospy.set_param('roboy_is_busy', busy)
        print("IS_BUSY:" + str(busy))

    # -------------------- states: conversation flow -------------------- #

    @rs.state(
        cond=sig_wait_for_telegram_customer_to_come_close,
        write=rawio.prop_out)
    def wait_for_telegram_customer(ctx: rs.ContextWrapper):
        time.sleep(0)   # TODO adjust to how much time Roboy should give a Telegram customer to come up
        ctx[rawio.prop_out] = verbaliser.get_random_successful_answer("greet_general")

    @rs.state(
        cond=interloc.prop_all.pushed().detached().min_age(2)
             | idle.sig_bored.min_age(10)
             | sig_suggested_ice_cream.min_age(20),
        read=prop_suggested_ice_cream,
        write=(rawio.prop_out, prop_suggested_ice_cream),
        signal=sig_suggested_ice_cream,
        emit_detached=True)
    def prompt_order(ctx: rs.ContextWrapper):
        has_already_asked = ctx[prop_suggested_ice_cream]
        if not has_already_asked:
            ctx[rawio.prop_out] = verbaliser.get_random_question('greet_general')
            ctx[prop_suggested_ice_cream] = True
            return rs.Emit(wipe=True)

    @rs.state(
        cond=sig_suggested_ice_cream.max_age(20).min_age(-1) & sig_yesno_detected | sig_changed_flavor_or_scoops,
        read=(nlp.prop_yesno, prop_flavors, prop_scoops),
        write=rawio.prop_out)
    def analyse_ice_cream_suggestion_answer(ctx: rs.ContextWrapper):
        if ctx[prop_scoops] or ctx[prop_flavors]:
            return rs.Resign()
        elif ctx[nlp.prop_yesno].yes():
            ctx[rawio.prop_out] = verbaliser.get_random_successful_answer("greet_general")
        elif ctx[nlp.prop_yesno].no():
            ctx[rawio.prop_out] = verbaliser.get_random_failure_answer("greet_general")

    @rs.state(
        cond=sig_changed_flavor_or_scoops | sig_ask_again_order,
        signal=sig_finish_order_question,
        emit_detached=True,
        read=(prop_flavor_scoop_tuple_list, prop_flavors, prop_scoops),
        write=(rawio.prop_out, prop_flavor_scoop_tuple_list, prop_flavors, prop_scoops, prop_order_verified))
    def check_scoops_flavor_combined(ctx: rs.ContextWrapper):
        if -1 in prop_scoops.read():
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("error_scoops")
            ctx[prop_flavors] = []
            ctx[prop_scoops] = []
        elif (prop_flavors.read() and prop_scoops.read() and len(prop_flavors.read()) == len(prop_scoops.read()))\
                or (not prop_flavors.read() and not prop_scoops.read() and prop_flavor_scoop_tuple_list.read()):
            current_order = [x for x in zip(prop_flavors.read(), prop_scoops.read())]
            add_orders_together(current_order, prop_flavor_scoop_tuple_list.read())
            ctx[prop_flavor_scoop_tuple_list] = current_order
            ctx[prop_flavors] = []
            ctx[prop_scoops] = []
            possibly_complete_order, _ = get_complete_order_and_cost(prop_flavor_scoop_tuple_list.read())
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("legit_order").format(order=possibly_complete_order)
            ctx[prop_order_verified] = True
            return rs.Emit(wipe=True)
        elif len(prop_flavors.read()) > len(prop_scoops.read()):
            current_order = [(prop_flavors.read()[i], prop_scoops.read()[i]) for i in range(0, len(prop_scoops.read()))]
            add_orders_together(current_order, prop_flavor_scoop_tuple_list.read())
            ctx[prop_flavor_scoop_tuple_list] = current_order
            ctx[prop_flavors] = prop_flavors.read()[len(prop_scoops.read()):]
            ctx[prop_scoops] = []
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("need_scoop") \
                .format(flavor=fix_pronunciation(prop_flavors.read()[0]))
            return rs.Emit(wipe=True)
        elif len(prop_flavors.read()) < len(prop_scoops.read()):
            current_order = [(prop_flavors.read()[i], prop_scoops.read()[i]) for i in
                             range(0, len(prop_flavors.read()))]
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
            return rs.Emit(wipe=True)



    @rs.state(
        cond=sig_finish_order_question.min_age(15).max_age(-1),
        read=prop_asked_order_count,
        write=(prop_asked_order_count, prop_flavor_scoop_tuple_list, prop_flavors,
               prop_scoops, prop_suggested_ice_cream, rawio.prop_out),
        signal=sig_ask_again_order,
        emit_detached=True)
    def waiting_for_complete_order(ctx: rs.ContextWrapper):
        asked_order_count = ctx[prop_asked_order_count]
        if asked_order_count < 3:
            ctx[prop_asked_order_count] = asked_order_count + 1
            return rs.Emit(wipe=True)
        else:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("order_aborted")
            ctx[prop_flavor_scoop_tuple_list] = []
            ctx[prop_flavors] = []
            ctx[prop_scoops] = []
            ctx[prop_suggested_ice_cream] = False
            ctx[prop_asked_order_count] = 1

    @rs.state(
        cond=sig_yesno_detected,
        read=(prop_flavor_scoop_tuple_list, nlp.prop_yesno, prop_order_verified),
        write=(rawio.prop_out, prop_price, prop_order_verified),
        signal=sig_wait_for_cup,
        emit_detached=True)
    def analyse_finish_order_answer(ctx: rs.ContextWrapper):
        if ctx[prop_order_verified] and ctx[nlp.prop_yesno].yes():
            flavor_scoop_tuple_list = ctx[prop_flavor_scoop_tuple_list]
            complete_order, complete_cost = get_complete_order_and_cost(flavor_scoop_tuple_list)
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("preparing_order").format(order=complete_order)
            ctx[prop_price] = complete_cost * 100  # price is in cents
            ctx[prop_order_verified] = False
            return rs.Emit(wipe=True)
        elif ctx[prop_order_verified] and ctx[nlp.prop_yesno].no():
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("continue_order")
            ctx[prop_order_verified] = False

    @rs.state(
        cond=sig_wait_for_cup,
        signal=sig_send_to_scooping)
    def while_customer_places_cup(ctx: rs.ContextWrapper):
        time.sleep(0)   # TODO adjust however long a customer should have to place the cup
        return rs.Emit()

    @rs.state(
        cond=sig_send_to_scooping.detached(),
        read=prop_flavor_scoop_tuple_list)
    def send_order_to_scooping(ctx: rs.ContextWrapper):
        if ROS_AVAILABLE:
            client.wait_for_server()
        scooping_communication.send_order(ctx[prop_flavor_scoop_tuple_list])

    @rs.state(
        cond=sig_send_to_scooping.detached() | sig_loop_feedback,
        write=rawio.prop_out,
        signal=sig_loop_feedback,
        emit_detached=True)
    def feedback_state(ctx: rs.ContextWrapper):
        if ROS_AVAILABLE and (client.get_result() is not None) \
                or (not ROS_AVAILABLE and scooping_communication.stop_feedback):
            return rs.Emit(wipe=True)
        if ROS_AVAILABLE and scooping_communication.feedback is not None:
            finished_scoops, status_message = scooping_communication.feedback
            if not scooping_communication.last_scooping_feedback_array == finished_scoops:
                scooping_communication.last_scooping_feedback_array = finished_scoops
                if status_message == "more time":
                    ctx[rawio.prop_out] = verbaliser.get_random_phrase("time")
                else:
                    scoops_left = finished_scoops.count(False)
                    if scoops_left == 1:
                        ctx[rawio.prop_out] = verbaliser.get_random_phrase("scoops_left").format(c=scoops_left, s="")
                    elif scoops_left > 1:
                        ctx[rawio.prop_out] = verbaliser.get_random_phrase("scoops_left").format(c=scoops_left, s="s")
        return rs.Emit(wipe=True)

    @rs.state(
        cond=sig_loop_feedback,
        read=prop_flavor_scoop_tuple_list,
        write=rawio.prop_out,
        signal=sig_start_payment,
        emit_detached=True)
    def after_scooping(ctx: rs.ContextWrapper):
        if (ROS_AVAILABLE and client.get_result() is None) \
                or (not ROS_AVAILABLE and not scooping_communication.stop_feedback):
            return rs.Resign()
        if ROS_AVAILABLE and client.get_result() is not None and not client.get_result().success:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("unexpected")  # TODO stop convo? output result.error?
        else:
            return rs.Emit(wipe=True)

    @rs.state(
        cond=sig_start_payment,
        read=(prop_flavor_scoop_tuple_list, prop_payment_option),
        write=rawio.prop_out,
        signal=sig_asked_payment_method,
        emit_detached=True)
    def ask_payment_method(ctx: rs.ContextWrapper):
        flavor_scoop_tuple_list = ctx[prop_flavor_scoop_tuple_list]
        complete_order, complete_cost = get_complete_order_and_cost(flavor_scoop_tuple_list)
        ctx[rawio.prop_out] = verbaliser.get_random_phrase("payment").format(cost=complete_cost, order=complete_order)
        return rs.Emit(wipe=True)

    @rs.state(
        cond=sig_asked_payment_method.min_age(15),
        read=prop_asked_payment_count,
        write=(prop_asked_payment_count, prop_asked_order_count, prop_flavor_scoop_tuple_list, prop_flavors,
               prop_scoops, prop_suggested_ice_cream, rawio.prop_out),
        signal=sig_start_payment,
        emit_detached=True)
    def waiting_for_payment_answer(ctx: rs.ContextWrapper):
        asked_payment_count = ctx[prop_asked_payment_count]
        if asked_payment_count < 3:
            ctx[prop_asked_payment_count] += 1
            return rs.Emit(wipe=True)
        else:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("no_payment")
            ctx[prop_flavor_scoop_tuple_list] = []
            ctx[prop_flavors] = []
            ctx[prop_scoops] = []
            ctx[prop_suggested_ice_cream] = False
            ctx[prop_asked_payment_count] = 1
            ctx[prop_asked_order_count] = 1

    @rs.state(
        cond=(sig_asked_payment_method.max_age(15).min_age(-1) & sig_changed_payment_option.detached())
             | sig_payment_incomplete,
        read=prop_payment_option,
        write=(rawio.prop_out, prop_asked_payment_count),
        signal=sig_insert_coins_or_scan_qr,
        emit_detached=True)
    def start_payment(ctx: rs.ContextWrapper):
        payment_option = ctx[prop_payment_option]
        if payment_option == PaymentOptions.PAYPAL:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("paypal_option")
            return rs.Emit(wipe=True)
        elif payment_option == PaymentOptions.COIN:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("coin_option")
            return rs.Emit(wipe=True)

    @rs.state(
        cond=sig_insert_coins_or_scan_qr,
        read=(prop_payment_option, prop_price, prop_flavor_scoop_tuple_list),
        write=(rawio.prop_out, prop_payment_success, prop_price),
        signal=sig_finished_payment,
        emit_detached=True)
    def payment_process(ctx: rs.ContextWrapper):
        payment_option = ctx[prop_payment_option]
        price = ctx[prop_price]
        amount_paid, error_message = payment_communication(price, payment_option, ctx[prop_flavor_scoop_tuple_list])
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
        return rs.Emit(wipe=True)

    @rs.state(
        cond=sig_finished_payment,
        read=prop_payment_success,
        write=(rawio.prop_out, prop_flavor_scoop_tuple_list, prop_flavors, prop_scoops, prop_suggested_ice_cream,
               prop_price, prop_payment_success, prop_payment_option, prop_asked_order_count, prop_asked_payment_count),
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
            ctx[prop_asked_order_count] = 1
            ctx[prop_asked_payment_count] = 1
        else:
            return rs.Emit(wipe=True)

    @rs.state(
       cond=interloc.prop_all.popped(),
       write=(prop_flavor_scoop_tuple_list, prop_flavors, prop_scoops, prop_suggested_ice_cream,
               prop_price, prop_payment_success, prop_payment_option, prop_asked_order_count, prop_asked_payment_count))
    def customer_left(ctx: rs.ContextWrapper):
        ctx[prop_flavor_scoop_tuple_list] = []
        ctx[prop_flavors] = []
        ctx[prop_scoops] = []
        ctx[prop_suggested_ice_cream] = False
        ctx[prop_price] = -1
        ctx[prop_payment_option] = -1
        ctx[prop_payment_success] = False
        ctx[prop_asked_order_count] = 1
        ctx[prop_asked_payment_count] = 1


# -------------------- functions outside module -------------------- #

def fix_pronunciation(flavor):
    if flavor == "vanilla" or flavor == "vanillas":
        return "vanillla"
    if flavor == "chocolate" or flavor == "chocolates" or flavor == "choco":
        return "choclate"
    if flavor == "strawberry" or flavor == "strawberries":
        return "strawbaary"


def get_complete_order_and_cost(flavor_scoop_tuple_list):
    order = ""
    cost = 0
    if len(flavor_scoop_tuple_list) == 1:
        if flavor_scoop_tuple_list[0][1] == 1:
            order += "{scoops} scoop of {flavor}".format(flavor=fix_pronunciation(flavor_scoop_tuple_list[0][0]),
                                                         scoops=flavor_scoop_tuple_list[0][1])
        else:
            order += "{scoops} scoops of {flavor}".format(flavor=fix_pronunciation(flavor_scoop_tuple_list[0][0]),
                                                          scoops=flavor_scoop_tuple_list[0][1])
        cost = cost_per_scoop * flavor_scoop_tuple_list[0][1]
    else:
        order_length = len(flavor_scoop_tuple_list)
        for i in range(0, order_length - 1):
            if flavor_scoop_tuple_list[i][1] == 1:
                order += "{scoops} scoop of {flavor}, ".format(flavor=fix_pronunciation(flavor_scoop_tuple_list[i][0]),
                                                               scoops=flavor_scoop_tuple_list[i][1])
            else:
                order += "{scoops} scoops of {flavor}, ".format(flavor=fix_pronunciation(flavor_scoop_tuple_list[i][0]),
                                                                scoops=flavor_scoop_tuple_list[i][1])
            cost += cost_per_scoop * flavor_scoop_tuple_list[i][1]
        order = order[:len(order) - 2]
        if flavor_scoop_tuple_list[order_length - 1][1] == 1:
            order += " and {scoops} scoop of {flavor}" \
                .format(flavor=fix_pronunciation(flavor_scoop_tuple_list[order_length - 1][0]),
                        scoops=flavor_scoop_tuple_list[order_length - 1][1])
        else:
            order += " and {scoops} scoops of {flavor}" \
                .format(flavor=fix_pronunciation(flavor_scoop_tuple_list[order_length - 1][0]),
                        scoops=flavor_scoop_tuple_list[order_length - 1][1])
        cost += cost_per_scoop * flavor_scoop_tuple_list[order_length - 1][1]
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
            if 0 < int(entity) < 10:
                scoops += [int(entity)]
            else:
                scoops += [-1]  # indicator that impossible amount was ordered
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
            if word2num.get(entity):
                scoops += [word2num[entity]]
            else:
                scoops += [-1]  # indicator that impossible amount was ordered
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
