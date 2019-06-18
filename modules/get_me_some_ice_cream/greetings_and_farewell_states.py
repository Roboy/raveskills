import ravestate as rs
import ravestate_rawio as rawio
from get_me_some_ice_cream.recognition import familiar_customer, get_customer_name
from get_me_some_ice_cream.utils import *
from ravestate_verbaliser import verbaliser
from os.path import realpath, dirname, join
import ravestate_nlp as nlp

sig_farewell = rs.Signal(name="farewell")


@rs.state(cond=rs.sig_startup, write="verbaliser:intent")
def greet_people(ctx):
    if not familiar_customer():
        ctx["verbaliser:intent"] = "greeting"


@rs.state(cond=rs.sig_startup, write=rawio.prop_out)
def greet_familiar_customer(ctx):
    if familiar_customer():
        verbaliser.add_file(join(dirname(realpath(__file__)), "important_phrases", "greet.yml"))
        out = verbaliser.get_random_phrase('greet_familiar_customer')
        edited_out = insert_user_name(out, get_customer_name())
        ctx[rawio.prop_out] = edited_out


@rs.state(
    cond=nlp.prop_tokens.changed(),
    read=(nlp.prop_tokens, nlp.prop_triples),
    signal=sig_farewell
)
def farewell(ctx):
    tokens = ctx[nlp.prop_tokens]
    farewell_list = ['bye', 'goodbye', 'ciao']
    if [token for token in tokens if token in farewell_list]:
        return rs.Emit()


@rs.state(cond=sig_farewell, write="verbaliser:intent")
def farewell_people(ctx):
    if not familiar_customer():
        ctx["verbaliser:intent"] = "farewell"


@rs.state(cond=sig_farewell, write=rawio.prop_out)
def farewell_familiar_customer(ctx):
    if familiar_customer():
        verbaliser.add_file(join(dirname(realpath(__file__)), "important_phrases", "farewell.yml"))
        out = verbaliser.get_random_phrase('farewell_familiar_customer')
        edited_out = insert_user_name(out, get_customer_name())
        ctx[rawio.prop_out] = edited_out
