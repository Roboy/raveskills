import ravestate as rs
import ravestate_nlp as nlp

from get_me_some_ice_cream.utils import extract_scoops

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

