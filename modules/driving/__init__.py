import ravestate as rs
import ravestate_nlp as nlp
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_idle as idle
import rospy
from roboy_cognition_msgs.srv import DriveToLocation
from ravestate_verbaliser import verbaliser
from os.path import realpath, dirname, join
verbaliser.add_folder(join(dirname(realpath(__file__))+"/phrases"))

DESIRE_SYNONYMS = {"want", "like", "desire", "have", "decide", "get", "choose", "wish", "prefer"}
NEGATION_SYNONYMS = {"no", "not"}
ICE_CREAM_SYNONYMS = {"icecream", "ice", "cream", "gelato", "sorbet"}

# TODO connect with data base from autonomous driving and possibly add synonyms for all places
PLACES = {"mensa", "mi", "mw", "ubahn"}
PROXIMITY_SYNONYMS = {"near", "close", "at", "right", "by", "in"}


with rs.Module(name="Luigi"):

    # -------------------- properties -------------------- #

    prop_suggested_ice_cream = rs.Property(name="suggested_ice_cream", always_signal_changed=False, default_value=False,
                                           allow_read=True, allow_write=True)
    prop_location = rs.Property(name="location", always_signal_changed=False, default_value=[], allow_read=True,
                                allow_write=True)

    # -------------------- signals -------------------- #

    sig_start_order_question = rs.Signal("start_order_question")
    sig_finish_order_question = rs.Signal("finish_order_question")
    sig_finished_payment = rs.Signal("finished_payment")
    sig_yesno_detected = rs.Signal("yesno")
    sig_changed_flavor_or_scoops = rs.Signal("changed_flavor_or_scoops")
    sig_telegram_conversation = rs.Signal("telegram_conversation")
    sig_ice_cream_desire = rs.Signal("ice_cream_desire")
    sig_suggested_ice_cream = rs.Signal("suggested_ice_cream")

    # -------------------- states: detection -------------------- #

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
        read=(nlp.prop_tokens, nlp.prop_triples, nlp.prop_lemmas),
        write=prop_location)
    def detect_location(ctx: rs.ContextWrapper):
        tokens = ctx[nlp.prop_tokens]
        triples = ctx[nlp.prop_triples]
        lemmas = ctx[nlp.prop_lemmas]
        if triples[0].match_either_lemma(pred=PLACES) and \
                not NEGATION_SYNONYMS & set(lemmas):
            # for expressing locations with just single words such as "mensa" or "mi building"
            ctx[prop_location] = extract_location(tokens)
        elif triples[0].match_either_lemma(subj={"i"}) and \
                triples[0].match_either_lemma(pred={"be"}) and \
                triples[0].match_either_lemma(obj=PLACES) and \
                not NEGATION_SYNONYMS & set(lemmas):
            # for expressing locations using sentences like
            # "i am in front of the mensa"
            # "i am near the mi"
            ctx[prop_location] = extract_location(tokens)
        elif triples[0].match_either_lemma(pred=PROXIMITY_SYNONYMS) and \
                triples[0].match_either_lemma(obj=PLACES) and \
                not NEGATION_SYNONYMS & set(lemmas):
            # for expressing locations using phrases like
            # "right by the mensa"
            # "at the mi"
            ctx[prop_location] = extract_location(tokens)
        elif triples[0].match_either_lemma(pred={"meet", "let"}) and \
                triples[0].match_either_lemma(obj=PLACES) and \
                not NEGATION_SYNONYMS & set(lemmas):
            # for expressing locations using phrases like
            # "let's meet at the ubahn"
            # "meet me at the mw"
            ctx[prop_location] = extract_location(tokens)

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
        cond=sig_suggested_ice_cream.max_age(-1) & sig_yesno_detected,
        read=nlp.prop_yesno,
        write=rawio.prop_out,
        signal=sig_start_order_question,
        emit_detached=True)
    def analyse_ice_cream_suggestion_answer(ctx: rs.ContextWrapper):
        if ctx[nlp.prop_yesno] == "yes":
            ctx[rawio.prop_out] = verbaliser.get_random_successful_answer("greet_general")
            return rs.Emit()
        elif ctx[nlp.prop_yesno] == "no":
            ctx[rawio.prop_out] = verbaliser.get_random_failure_answer("greet_general")

    @rs.state(
        cond=sig_start_order_question.max_age(-1),
        write=rawio.prop_out)
    def ask_for_location(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = verbaliser.get_random_question("location")


    @rs.state(
        read=prop_location,
        write=rawio.prop_out)
    def known_location(ctx: rs.ContextWrapper):
        location = ctx[prop_location]
        if location == "unknown":
            ctx[rawio.prop_out] = verbaliser.get_random_failure_answer("location")
        else:
            eta, path_found, error_msg = ad_communication(location)
            if path_found:
                ctx[rawio.prop_out] = verbaliser.get_random_successful_answer("location") \
                    .format(location=location, min=eta)
            else:
                ctx[rawio.prop_out] = verbaliser.get_random_phrase("no_path")


# -------------------- functions outside module -------------------- #

def extract_location(prop_tokens):
    for token in prop_tokens:
        if token in PLACES:
            return token
    return "unknown"


def ad_communication(location):
    rospy.wait_for_service('autonomous_driving')
    try:
        drive_to_location = rospy.ServiceProxy('autonomous_driving', DriveToLocation)
        response = drive_to_location(location)
        return response.eta, response.path_found, response.error_message
    except rospy.ROSInterruptException as e:
        print('Service call failed:', e)
    # If driving module is run without ROS, comment everything from above (including imports) and uncomment this:
    # return 42, True, ""
