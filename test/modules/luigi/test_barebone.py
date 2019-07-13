import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_idle as idle
import ravestate_ontology
import ravestate_verbaliser as verbaliser
import luigi

from reggol import get_logger, set_default_loglevel
logger = get_logger(__name__)


def test_run_luigi():
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

    assert luigi.detect_flavors_and_scoops.wait(0)


if __name__ == "__main__":
    # from hanging_threads import start_monitoring
    # monitoring_thread = start_monitoring()
    set_default_loglevel("DEBUG")
    test_run_luigi()
    exit()