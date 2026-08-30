"""RTD relative-position processor (code-only double differences)."""

from .rtk_processor import RtkProcessor


class RtdProcessor(RtkProcessor):
    """Relative positioning processor using pseudorange only."""

    def __init__(self, nav):
        super().__init__(nav)
        self._use_phase = False
        nav.use_phase = False
        relpos = self._relpos
        self._relpos = lambda nav_, obsr, obsb, sol: relpos(
            nav_, obsr, obsb, sol, use_phase=False
        )
