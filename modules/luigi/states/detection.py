import ravestate as rs
import ravestate_nlp as nlp
from luigi.luigi import mod

prop_flavor = rs.Property(name="flavor", allow_read=True, allow_write=True, always_signal_changed=True)
prop_scoops = rs.Property(name="scoops", allow_read=True, allow_write=True, always_signal_changed=True)


@rs.state(
    cond=nlp.prop_tokens.changed(),
    read=(nlp.prop_tokens, nlp.prop_triples, nlp.prop_lemmas),
    write=prop_flavor,
    signal=prop_flavor.changed_signal)
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
        # TODO this part needs to be changed when ordering multiple flavors at the same time (possibly also with scoops)
        if "chocolate" in tokens:
            # prop_flavor.write("chocolate")
            ctx[prop_flavor] = "chocolate"
            return rs.Emit()
        if "vanilla" in tokens:
            # prop_flavor.write("vanilla")
            ctx[prop_flavor] = "vanilla"
            return rs.Emit()


@rs.state(
    cond=nlp.prop_ner.changed(),
    read=nlp.prop_ner,
    write=prop_scoops,
    signal=prop_scoops.changed_signal)
def detect_scoops(ctx: rs.ContextWrapper):
    detected_scoops = extract_scoops(ctx[nlp.prop_ner])
    if detected_scoops is not None:
        # prop_scoops.write(detected_scoops)
        ctx[prop_scoops] = detected_scoops
        return rs.Emit()


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


mod.add(prop_flavor)
mod.add(prop_scoops)
mod.add(detect_flavor)
mod.add(detect_scoops)
