import ravestate as rs
import ravestate_rawio as rawio

with rs.Module(name="fourtytwo"):

    meaning_of_life_signal = rs.Signal(name="meaning of life")

    @rs.state(read=rawio.prop_in, signal=meaning_of_life_signal)
    def meaning_of_life_state(ctx):
        text = ctx[rawio.prop_in]
        if "meaning" in text and "life" in text:
            return rs.Emit()

    @rs.state(cond=meaning_of_life_signal, write=rawio.prop_out)
    def fourty_two_state(ctx):
        ctx[rawio.prop_out] = "42"
