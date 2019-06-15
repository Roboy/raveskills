import ravestate as rs
import ravestate_nlp as nlp
import ravestate_rawio as rawio

with rs.Module(name="icecream"):

    sig_flavor = rs.Signal(name="flavor")
    sig_payment = rs.Signal(name="payment")
    sig_order = rs.Signal(name="order")
    sig_flavor_order = rs.Signal(name="flavor order")
    sig_payment_request = rs.Signal(name="payment request")

    @rs.state(
        cond=nlp.prop_tokens.changed(),
        read=nlp.prop_tokens,
        signal=sig_flavor)
    def flavor_signal(ctx: rs.ContextWrapper):
        tokens = ctx[nlp.prop_tokens]
        if "chocolate" in tokens or "vanilla" in tokens:
            return rs.Emit()

    @rs.state(
        cond=nlp.prop_tokens.changed(),
        read=nlp.prop_tokens,
        signal=sig_payment)
    def payment_signal(ctx: rs.ContextWrapper):
        tokens = ctx[nlp.prop_tokens]
        if "bitcoin" in tokens or "credit" in tokens or "cash" in tokens:
            return rs.Emit()

    @rs.state(
        cond=nlp.prop_triples.changed(),
        read=nlp.prop_triples,
        signal=sig_order)
    def order_signal(ctx: rs.ContextWrapper):
        triples = ctx[nlp.prop_triples]
        if triples[0].match_either_lemma(pred={"want", "like", "desire"}):
            return rs.Emit()

    @rs.state(
        cond=nlp.prop_triples.changed(),
        read=nlp.prop_triples,
        signal=sig_flavor_order)
    def wants_flavor_signal(ctx: rs.ContextWrapper):
        triples = ctx[nlp.prop_triples]
        # TODO also include that the object does not exactly have to match chocolate or vanilla but can simply include
        # a flavor. E.g. "I would like some chocolate ice cream, please" should also be detected.
        if triples[0].match_either_lemma(pred={"want", "like", "desire"}) \
                and triples[0].match_either_lemma(obj={"chocolate", "vanilla"}):
            return rs.Emit()

    @rs.state(
        cond=nlp.prop_triples.changed(),
        read=nlp.prop_triples,
        signal=sig_payment_request)
    def wants_flavor_signal(ctx: rs.ContextWrapper):
        triples = ctx[nlp.prop_triples]
        # TODO also include questions like "Can I pay with credit card, please?"
        if triples[0].match_either_lemma(pred={"want", "like", "pay"}) \
                and triples[0].match_either_lemma(obj={"bitcoin", "cash", "credit"}):
            return rs.Emit()

    # TODO if I leave these in, sometimes f. ex. "I want vanilla" is detected as "flavor" instead of "flavor order"
    #
    # @rs.state(
    #     cond=sig_flavor,
    #     write=rawio.prop_out)
    # def flavor_received(ctx: rs.ContextWrapper):
    #     ctx[rawio.prop_out] = "Flavor"
    #
    # @rs.state(
    #     cond=sig_order,
    #     write=rawio.prop_out)
    # def order_received(ctx: rs.ContextWrapper):
    #     ctx[rawio.prop_out] = "Order"
    #
    # @rs.state(
    #     cond=sig_payment,
    #     write=rawio.prop_out)
    # def payment_received(ctx: rs.ContextWrapper):
    #     ctx[rawio.prop_out] = "Payment"

    @rs.state(
        cond=sig_flavor_order,
        write=rawio.prop_out)
    def flavor_order_received(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = "Flavor Order"

    @rs.state(
        cond=sig_payment_request,
        write=rawio.prop_out)
    def payment_request_received(ctx: rs.ContextWrapper):
        ctx[rawio.prop_out] = "Payment Request"

    # TODO possibly alternative? Does not feel as stable as with sig_flavor_order but might allow for more flexibility?
    #
    # @rs.state(
    #     cond=sig_flavor and sig_order,
    #     write=rawio.prop_out)
    # def flavor_order_received_conjunction(ctx: rs.ContextWrapper):
    #     ctx[rawio.prop_out] = "Flavor Order CONJUNCTION"
