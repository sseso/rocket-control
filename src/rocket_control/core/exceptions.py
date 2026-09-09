"""Library-level exceptions. Physics helpers must not call sys.exit."""


class InfeasibleError(Exception):
    """Raised when a requested manoeuvre fails a feasibility check."""
