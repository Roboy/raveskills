import ravestate as rs
import ravestate_nlp as nlp

prop_flavor = rs.Property(name="flavor", allow_read=True, allow_write=True, always_signal_changed=True)
prop_scoops = rs.Property(name="scoops", allow_read=True, allow_write=True, always_signal_changed=True)


@rs.state(
    cond=nlp.prop_tokens.changed(),
    read=nlp.prop_tokens,
    signal=prop_flavor.changed_signal)
def detect_flavor(ctx: rs.ContextWrapper):
    tokens = ctx[nlp.prop_tokens]
    if "chocolate" in tokens:
        prop_flavor.write("chocolate")
        return rs.Emit()
    if "vanilla" in tokens:
        prop_flavor.write("vanilla")
        return rs.Emit()


@rs.state(
    cond=nlp.prop_ner.changed(),
    read=nlp.prop_ner,
    signal=prop_scoops.changed_signal
)
def detect_scoops(ctx: rs.ContextWrapper):
    detected_scoops = extract_scoops(ctx[nlp.prop_ner])
    if detected_scoops is not None:
        prop_scoops.write(detected_scoops)
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
