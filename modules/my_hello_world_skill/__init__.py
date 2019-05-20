import ravestate as rs
import ravestate_rawio as rawio


with rs.Module(name="test_skill"):

    @rs.state(read=rawio.prop_in, write=rawio.prop_out)
    def my_state(ctx):
        ctx[rawio.prop_out] = f"I recognized {len(ctx[rawio.prop_in])} characters!"
