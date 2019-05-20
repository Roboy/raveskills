import ravestate as rs
import ravestate_rawio as rawio
from roboy_parlai import wildtalk
import re

with rs.Module(name="fourtytwo"):

    fix_spaces = re.compile(r'\s*([?!.,]+(?:\s+[?!.,]+)*)\s*')

    @rs.state(cond=rawio.prop_in.changed().max_age(-1.), read=rawio.prop_in, write=rawio.prop_out)
    def fourty_two_state(ctx):
        text = ctx[rawio.prop_in]
        # make sure text is not empty
        if not text:
            return rs.Resign()
        # check whether text is about the meaning of life and if so return 42, otherwise talk wild
        if re.search('meaning', text) is not None and re.search('life', text) is not None:
            result = "42"
        else:
            result = wildtalk(text)
        result = fix_spaces.sub(lambda x: "{} ".format(x.group(1).replace(" ", "")), result)
        ctx[rawio.prop_out] = result
