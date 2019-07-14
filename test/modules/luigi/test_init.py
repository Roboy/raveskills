import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_idle as idle
import ravestate_ontology
import ravestate_verbaliser as verbaliser
import ravestate_nlp as nlp
import luigi

from reggol import get_logger, set_default_loglevel

logger = get_logger(__name__)

from os.path import realpath, dirname, join

verbaliser.add_folder(join(dirname(realpath(__file__)) + "/phrases"))

#TODO make an assertion after test is finished in decline offer,(after decline offer test it is stuck on infinite loop)

# def test_decline_offer():
#     last_output = ""
#
#     with rs.Module(name="luigi_test"):
#         @rs.state(cond=rs.sig_startup, read=interloc.prop_all)
#         def luigi_hi(ctx: rs.ContextWrapper):
#             ravestate_ontology.initialized.wait()
#             interloc.handle_single_interlocutor_input(ctx, "hi")
#
#         @rs.state(read=rawio.prop_out)
#         def raw_out(ctx: rs.ContextWrapper):
#             nonlocal last_output
#             last_output = ctx[rawio.prop_out]
#             logger.info(f"Output: {ctx[rawio.prop_out]}")
#
#     ctx = rs.Context(
#         "rawio",
#         "ontology",
#         "idle",
#         "interloc",
#         "nlp",
#         "Luigi",
#         "luigi_test"
#     )
#
#     @rs.receptor(ctx_wrap=ctx, write=rawio.prop_in)
#     def say(ctx: rs.ContextWrapper, what: str):
#         ctx[rawio.prop_in] = what
#
#     ctx.emit(rs.sig_startup)
#     ctx.run_once()
#
#     assert luigi_hi.wait()
#
#     # Wait for greeting
#     while not raw_out.wait(.1):
#         ctx.run_once()
#     assert last_output in verbaliser.get_question_list("greet_general")
#
#     say("no")
#
#     # Wait for acknowledgement of order
#     while not raw_out.wait(.1):
#         ctx.run_once()
#
#     assert last_output in verbaliser.verbaliser.get_random_failure_answer("greet_general")
#
#     say("bye")
#
#     # Wait for acknowledgement of answer
#     while not raw_out.wait(.1):
#         ctx.run_once()

def test_legit_order():
    last_output = ""

    with rs.Module(name="luigi_test"):

        @rs.state(cond=rs.sig_startup, read=interloc.prop_all)
        def luigi_hi(ctx: rs.ContextWrapper):
            ravestate_ontology.initialized.wait()
            interloc.handle_single_interlocutor_input(ctx, "hi")

        @rs.state(read=rawio.prop_out)
        def raw_out(ctx: rs.ContextWrapper):
            nonlocal last_output
            last_output = ctx[rawio.prop_out]
            logger.info(f"Output: {ctx[rawio.prop_out]}")

    ctx = rs.Context(
        "rawio",
        "ontology",
        "idle",
        "interloc",
        "nlp",
        "Luigi",
        "luigi_test"
    )

    @rs.receptor(ctx_wrap=ctx, write=rawio.prop_in)
    def say(ctx: rs.ContextWrapper, what: str):
        ctx[rawio.prop_in] = what

    ctx.emit(rs.sig_startup)
    ctx.run_once()

    assert luigi_hi.wait()

    # Wait for greeting
    while not raw_out.wait(.1):
        ctx.run_once()
    assert last_output in verbaliser.get_question_list("greet_general")

    # Legit order

    say("three scoops of vanilla please")

    # Wait for acknowledgement of order
    while not raw_out.wait(.1):
        ctx.run_once()

    assert luigi.detect_flavors_and_scoops.wait(0)
    assert last_output.replace("3 scoops of vanilla", "{order}") in verbaliser.get_phrase_list("legit_order")

    say("yes")

    # Wait for acknowledgement of answer
    while not raw_out.wait(.1):
        ctx.run_once()


def test_need_scoop():
    last_output = ""

    with rs.Module(name="luigi_test"):

        @rs.state(cond=rs.sig_startup, read=interloc.prop_all)
        def luigi_hi(ctx: rs.ContextWrapper):
            ravestate_ontology.initialized.wait()
            interloc.handle_single_interlocutor_input(ctx, "hi")

        @rs.state(read=rawio.prop_out)
        def raw_out(ctx: rs.ContextWrapper):
            nonlocal last_output
            last_output = ctx[rawio.prop_out]
            logger.info(f"Output: {ctx[rawio.prop_out]}")

    ctx = rs.Context(
        "rawio",
        "ontology",
        "idle",
        "interloc",
        "nlp",
        "Luigi",
        "luigi_test"
    )

    @rs.receptor(ctx_wrap=ctx, write=rawio.prop_in)
    def say(ctx: rs.ContextWrapper, what: str):
        ctx[rawio.prop_in] = what

    ctx.emit(rs.sig_startup)
    ctx.run_once()

    assert luigi_hi.wait()

    # Wait for greeting
    while not raw_out.wait(.1):
        ctx.run_once()
    assert last_output in verbaliser.get_question_list("greet_general")

    # Only give flavor

    say("I want vanilla please")

    # Wait for acknowledgement of flavor
    while not raw_out.wait(.1):
        ctx.run_once()
    assert last_output.replace("vanilla", "{flavor}") in verbaliser.get_phrase_list("need_scoop")

    # Give scoop count

    say("three")

    # Wait for acknowledgement of scoop
    while not raw_out.wait(.1):
        ctx.run_once()
    assert last_output.replace("3 scoops of vanilla", "{order}") in \
           verbaliser.get_phrase_list("legit_order")

    # Order is finished

    say("yes")

    # Wait for acknowledgement of answer
    while not raw_out.wait(.1):
        ctx.run_once()


def test_need_flavor():
    last_output = ""

    with rs.Module(name="luigi_test"):

        @rs.state(cond=rs.sig_startup, read=interloc.prop_all)
        def luigi_hi(ctx: rs.ContextWrapper):
            ravestate_ontology.initialized.wait()
            interloc.handle_single_interlocutor_input(ctx, "hi")

        @rs.state(read=rawio.prop_out)
        def raw_out(ctx: rs.ContextWrapper):
            nonlocal last_output
            last_output = ctx[rawio.prop_out]
            logger.info(f"Output: {ctx[rawio.prop_out]}")

    ctx = rs.Context(
        "rawio",
        "ontology",
        "idle",
        "interloc",
        "nlp",
        "Luigi",
        "luigi_test"
    )

    @rs.receptor(ctx_wrap=ctx, write=rawio.prop_in)
    def say(ctx: rs.ContextWrapper, what: str):
        ctx[rawio.prop_in] = what

    ctx.emit(rs.sig_startup)
    ctx.run_once()

    assert luigi_hi.wait()

    # Wait for name being asked
    while not raw_out.wait(.1):
        ctx.run_once()
    assert last_output in verbaliser.get_question_list("greet_general")

    # Only give one scoop

    say("one")

    # Wait for acknowledgement of answer
    while not raw_out.wait(.1):
        ctx.run_once()

    assert last_output.replace("scoop", "scoop{s}").replace("1", "{scoop}") in verbaliser.get_phrase_list("need_flavor")

    say("vanilla")

    # Wait for acknowledgement of scoop
    while not raw_out.wait(.1):
        ctx.run_once()

    assert last_output.replace("1 scoop of vanilla", "{order}") in verbaliser.get_phrase_list("legit_order")

    # Only give four scoop

    say("no")

    # Wait for acknowledgement of answer
    while not raw_out.wait(.1):
        ctx.run_once()

    say("four")

    # Wait for acknowledgement of answer
    while not raw_out.wait(.1):
        ctx.run_once()

    assert last_output.replace("scoops", "scoop{s}").replace("4", "{scoop}") in verbaliser.get_phrase_list(
        "need_flavor")

    say("chocolate")

    # Wait for acknowledgement of scoop
    while not raw_out.wait(.1):
        ctx.run_once()

    assert last_output.replace("4 scoops of chocolate and 1 scoop of vanilla", "{order}") \
           in verbaliser.get_phrase_list("legit_order")

    say("yes")

    # Wait for acknowledgement of scoop
    while not raw_out.wait(.1):
        ctx.run_once()


def test_legit_order_need_scoop_get_order_farewell():
    last_output = ""

    with rs.Module(name="luigi_test"):

        @rs.state(cond=rs.sig_startup, read=interloc.prop_all)
        def luigi_hi(ctx: rs.ContextWrapper):
            ravestate_ontology.initialized.wait()
            interloc.handle_single_interlocutor_input(ctx, "hi")

        @rs.state(read=rawio.prop_out)
        def raw_out(ctx: rs.ContextWrapper):
            nonlocal last_output
            last_output = ctx[rawio.prop_out]
            logger.info(f"Output: {ctx[rawio.prop_out]}")

    ctx = rs.Context(
        "rawio",
        "ontology",
        "idle",
        "interloc",
        "nlp",
        "Luigi",
        "luigi_test"
    )

    @rs.receptor(ctx_wrap=ctx, write=rawio.prop_in)
    def say(ctx: rs.ContextWrapper, what: str):
        ctx[rawio.prop_in] = what

    ctx.emit(rs.sig_startup)
    ctx.run_once()

    assert luigi_hi.wait()

    # Wait for greeting
    while not raw_out.wait(.1):
        ctx.run_once()
    assert last_output in verbaliser.get_question_list("greet_general")

    # Legit order

    say("chocolate two")

    # Wait for acknowledgement of order
    while not raw_out.wait(.1):
        ctx.run_once()

    assert luigi.detect_flavors_and_scoops.wait(0)
    assert last_output.replace("2 scoops of chocolate", "{order}") in verbaliser.get_phrase_list("legit_order")

    say("no")

    # Wait for acknowledgement of answer
    while not raw_out.wait(.1):
        ctx.run_once()

    # Only give flavor

    say("vanilla")

    # Wait for acknowledgement of flavor
    while not raw_out.wait(.1):
        ctx.run_once()
    assert last_output.replace("vanilla", "{flavor}") in verbaliser.get_phrase_list("need_scoop")

    # Only give scoops

    say("three")

    # Wait for acknowledgement of scoop
    while not raw_out.wait(.1):
        ctx.run_once()
    assert last_output.replace("3 scoops of vanilla and 2 scoops of chocolate", "{order}") in \
           verbaliser.get_phrase_list("legit_order")

    # Order is finished

    say("yes")

    # Wait for acknowledgement of answer
    while not raw_out.wait(.1):
        ctx.run_once()
    assert last_output.replace("5", "{cost}").replace("3 scoops of vanilla and 2 scoops of chocolate", "{order}") in \
           verbaliser.get_phrase_list("payment")

    # TODO Test payment

    say("bye")

    # Wait for acknowledgement of answer
    while not raw_out.wait(.1):
        ctx.run_once()
    assert last_output in verbaliser.get_phrase_list("luigi_farewell")


if __name__ == "__main__":
    # from hanging_threads import start_monitoring
    # monitoring_thread = start_monitoring()
    set_default_loglevel("DEBUG")
    test_legit_order_need_scoop_get_order_farewell()
    test_need_flavor()
    exit()
