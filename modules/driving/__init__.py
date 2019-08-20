import ravestate as rs
import ravestate_nlp as nlp
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_idle as idle
import asyncio
import websockets
import pickle
from ravestate_verbaliser import verbaliser
from os.path import realpath, dirname, join
from luigi.ws_communication import current_location

verbaliser.add_folder(join(dirname(realpath(__file__))+"/phrases"))

DESIRE_SYNONYMS = {"want", "like", "desire", "have", "decide", "get", "choose", "wish", "prefer"}
NEGATION_SYNONYMS = {"no", "not"}
ICE_CREAM_SYNONYMS = {"icecream", "ice", "cream", "gelato", "sorbet"}

# TODO connect with data base from autonomous driving and possibly add synonyms for all places
PLACES = {"mensa", "mi", "mw", "ubahn"}
PROXIMITY_SYNONYMS = {"near", "close", "at", "right", "by", "in"}

eta = ""
path_found = False
ws = 'ws://localhost:8765'  # TODO change to cloud address

with rs.Module(name="Luigi"):

    # -------------------- properties -------------------- #

    prop_suggested_ice_cream = rs.Property(name="suggested_ice_cream", always_signal_changed=False, default_value=False,
                                           allow_read=True, allow_write=True)
    prop_location = rs.Property(name="location", always_signal_changed=False, default_value=[], allow_read=True,
                                allow_write=True)
    prop_busy = rs.Property(name="busy", always_signal_changed=False, default_value=False, allow_read=True,
                                allow_write=True)

    # -------------------- signals -------------------- #

    sig_location_question = rs.Signal("location_question")
    sig_yesno_detected = rs.Signal("yesno")
    sig_ice_cream_desire_and_not_location = rs.Signal("ice_cream_desire_and_not_location")
    sig_ice_cream_desire_and_location = rs.Signal("ice_cream_desire_and_location")
    sig_suggested_ice_cream = rs.Signal("suggested_ice_cream")
    sig_asked_for_location = rs.Signal("asked_for_location")
    sig_decide_to_call_customer = rs.Signal("decide_to_call_customer")

    # -------------------- detection methods -------------------- #

    def detect_triple_ice_cream_desire(ctx):
        triples = ctx[nlp.prop_triples]
        lemmas = ctx[nlp.prop_lemmas]
        ice_cream_desire = False
        for i in range(0, len(triples)):
            if triples[i].match_either_lemma(subj={"i"}) and \
                    triples[i].match_either_lemma(pred=DESIRE_SYNONYMS) and \
                    triples[i].match_either_lemma(obj=ICE_CREAM_SYNONYMS) and \
                    not NEGATION_SYNONYMS & set(lemmas):
                # signal is emitted when customer expresses the wish for ice cream using phrases like
                # "i would like to have ice cream please"
                # "can i get some ice cream?"
                # "i want ice cream!"
                ice_cream_desire = True
        return ice_cream_desire

    def detect_triple_location(ctx):
        tokens = ctx[nlp.prop_tokens]
        triples = ctx[nlp.prop_triples]
        lemmas = ctx[nlp.prop_lemmas]
        location = ""
        for i in range(0, len(triples)):
            if triples[i].match_either_lemma(pred=PLACES) and \
                    not NEGATION_SYNONYMS & set(lemmas):
                # for expressing locations with just single words such as "mensa" or "mi building"
                location = extract_location(tokens)
            elif triples[i].match_either_lemma(subj={"i"}) and \
                    triples[i].match_either_lemma(pred={"be"}) and \
                    triples[i].match_either_lemma(obj=PLACES) and \
                    not NEGATION_SYNONYMS & set(lemmas):
                # for expressing locations using sentences like
                # "i am in front of the mensa"
                # "i am near the mi"
                location = extract_location(tokens)
            elif triples[i].match_either_lemma(pred=PROXIMITY_SYNONYMS) and \
                    triples[i].match_either_lemma(obj=PLACES) and \
                    not NEGATION_SYNONYMS & set(lemmas):
                # for expressing locations using phrases like
                # "right by the mensa"
                # "at the mi"
                location = extract_location(tokens)
            elif triples[i].match_either_lemma(pred={"meet", "let", "come"}) and \
                    not NEGATION_SYNONYMS & set(lemmas):
                # for expressing locations using phrases like
                # "let's meet at the ubahn"
                # "meet me at the mw"
                # "can you come to mensa"
                location = extract_location(tokens)
        return location

    # -------------------- detection states -------------------- #

    @rs.state(
        cond=nlp.prop_tokens.changed(),
        read=(nlp.prop_triples, nlp.prop_tokens, nlp.prop_lemmas),
        signal=sig_ice_cream_desire_and_not_location)
    def detect_ice_cream_desire_and_not_location(ctx: rs.ContextWrapper):
        ice_cream_desire = detect_triple_ice_cream_desire(ctx)
        location = detect_triple_location(ctx)
        if ice_cream_desire and not location:
            return rs.Emit(wipe=True)

    @rs.state(
        cond=nlp.prop_tokens.changed(),
        read=(nlp.prop_triples, nlp.prop_tokens, nlp.prop_lemmas),
        write=prop_location,
        signal=sig_ice_cream_desire_and_location)
    def detect_ice_cream_desire_and_location(ctx: rs.ContextWrapper):
        ice_cream_desire = detect_triple_ice_cream_desire(ctx)
        location = detect_triple_location(ctx)
        if ice_cream_desire and location:
            ctx[prop_location] = location
            return rs.Emit(wipe=True)

    @rs.state(
        cond=nlp.prop_tokens.changed() & sig_asked_for_location.detached().max_age(-1),
        read=(nlp.prop_tokens, nlp.prop_triples, nlp.prop_lemmas),
        write=prop_location,
        signal=sig_ice_cream_desire_and_location,
        emit_detached=True)
    def detect_location(ctx: rs.ContextWrapper):
        location = detect_triple_location(ctx)
        if location:
            ctx[prop_location] = location
            return rs.Emit(wipe=True)

    @rs.state(
        read=nlp.prop_yesno,
        signal=sig_yesno_detected)
    def yesno_detection(ctx: rs.ContextWrapper):
        if ctx[nlp.prop_yesno].yes() or ctx[nlp.prop_yesno].no():
            return rs.Emit(wipe=True)


    # -------------------- states: conversation flow -------------------- #

    @rs.state(
        cond=interloc.prop_all.pushed().detached().min_age(2)
             | idle.sig_bored.min_age(1)
             | sig_suggested_ice_cream.min_age(12),
        read=prop_suggested_ice_cream,
        write=(rawio.prop_out, prop_suggested_ice_cream),
        signal=sig_suggested_ice_cream,
        emit_detached=True)
    def prompt_order(ctx: rs.ContextWrapper):
        has_already_asked = ctx[prop_suggested_ice_cream]
        if not has_already_asked:
            ctx[rawio.prop_out] = verbaliser.get_random_question('greet_general')
            return rs.Emit(wipe=True)

    @rs.state(
        cond=sig_ice_cream_desire_and_not_location,
        read=prop_suggested_ice_cream,
        write=(rawio.prop_out, prop_suggested_ice_cream),
        signal=sig_decide_to_call_customer,
        emit_detached=True)
    def react_to_ice_cream_desire_and_no_location(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = verbaliser.get_random_successful_answer('greet_general')
        ctx[prop_suggested_ice_cream] = True
        return rs.Emit(wipe=True)

    @rs.state(
        cond=sig_suggested_ice_cream.max_age(12) & sig_yesno_detected,
        read=nlp.prop_yesno,
        write=rawio.prop_out,
        signal=sig_decide_to_call_customer,
        emit_detached=True)
    def analyse_ice_cream_suggestion_answer(ctx: rs.ContextWrapper):
        if ctx[nlp.prop_yesno].yes():
            ctx[rawio.prop_out] = verbaliser.get_random_successful_answer("greet_general")
            return rs.Emit(wipe=True)
        elif ctx[nlp.prop_yesno].no():
            ctx[rawio.prop_out] = verbaliser.get_random_failure_answer("greet_general")

    @rs.state(
        cond=sig_decide_to_call_customer,
        read=prop_busy,
        write=rawio.prop_out,
        signal=sig_location_question,)
    def ask_location_if_not_busy(ctx: rs.ContextWrapper):
        if ctx[prop_busy]:
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("busy").replace('{current_location}', current_location)
        else:
            return rs.Emit(wipe=True)

    @rs.state(
        cond=sig_location_question.max_age(-1),
        write=rawio.prop_out,
        signal=sig_asked_for_location,
        emit_detached=True)
    def ask_for_location(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = verbaliser.get_random_question("location_qa")
        return rs.Emit(wipe=True)

    @rs.state(
        cond=sig_ice_cream_desire_and_location,
        read=(prop_location, prop_busy,),
        write=(rawio.prop_out, prop_suggested_ice_cream))
    def known_location(ctx: rs.ContextWrapper):
        ctx[prop_suggested_ice_cream] = True
        if ctx[prop_busy]:
            current_location = None
            ctx[rawio.prop_out] = verbaliser.get_random_phrase("busy").replace('{current_location}', current_location)
        global eta
        location = ctx[prop_location]
        if location == "unknown":
            ctx[rawio.prop_out] = verbaliser.get_random_failure_answer("location_qa")
        else:
            communication_with_cloud(ws, location)
            if path_found:
                ctx[rawio.prop_out] = verbaliser.get_random_successful_answer("location_qa") \
                    .format(location=location, min=eta)
            else:
                ctx[rawio.prop_out] = verbaliser.get_random_phrase("no_path")

# -------------------- functions outside module -------------------- #


def extract_location(prop_tokens):
    for token in prop_tokens:
        if token in PLACES:
            return token
    return "unknown"


def communication_with_cloud(server, location="unknown"):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    asyncio.get_event_loop().run_until_complete(say(server, location))
    asyncio.get_event_loop().run_until_complete(listen(server))
    loop.close()


async def say(server, location):
    async with websockets.connect(server+'/pub') as websocket:
        location_encoding = pickle.dumps(location)
        await websocket.send(location_encoding)


async def listen(server):
    global eta
    global path_found
    async with websockets.connect(server+'/sub') as websocket:
        eta_encoding = await websocket.recv()
        path_found_encoding = await websocket.recv()
        eta = pickle.loads(eta_encoding, encoding='bytes')
        path_found = pickle.loads(path_found_encoding, encoding='bytes')
