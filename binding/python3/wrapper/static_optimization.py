from typing import TYPE_CHECKING, Generator

if TYPE_CHECKING:
    from .biorbd_model import Biorbd
from .misc import BiorbdArray, to_biorbd_array_input, to_biorbd_array_output
from ..biorbd import GeneralizedCoordinates, GeneralizedVelocity, GeneralizedTorque

try:
    from ..biorbd import StaticOptimization as StaticOptimizationBiorbd

    has_static_optimization = True
except ImportError:
    has_static_optimization = False


class StaticOptimization:
    def __init__(self, model: "Biorbd"):
        if not has_static_optimization:
            raise RuntimeError("In order to use StaticOptimization, biorbd must be compiled with STATIC_OPTIM=ON")

        self._model = model

    def perform_frames(
        self,
        all_q: BiorbdArray,
        all_qdot: BiorbdArray,
        all_tau: BiorbdArray,
        use_linearized_state: bool = False,
    ) -> Generator[BiorbdArray, None, None]:
        """
        Compute the muscle activations that reproduce the target generalized torques for all the frames.

        Parameters
        ----------
        all_q: BiorbdArray
            The generalized coordinates of each frame
        all_qdot: BiorbdArray
            The generalized velocities of each frame
        all_tau: BiorbdArray
            The generalized torques to reproduce at each frame
        use_linearized_state: bool
            If True, the faster linearized version of the solver is used. WARNING: the linearized solver of the
            current C++ core does not include the target torque in its constraint (see issue #393), so the
            solution may be far from the target. The default (False) uses the exact (non-linearized) solver,
            which reproduces the target torque but is slower.

        Returns
        -------
        A generator of the muscle activations, one per frame
        """
        q = [GeneralizedCoordinates(to_biorbd_array_input(q)) for q in all_q]
        qdot = [GeneralizedVelocity(to_biorbd_array_input(q)) for q in all_qdot]
        tau = [GeneralizedTorque(to_biorbd_array_input(t)) for t in all_tau]

        optim = StaticOptimizationBiorbd(self._model.internal, q, qdot, tau)
        optim.run(use_linearized_state)
        muscles_activations = optim.finalSolution()

        for activation in muscles_activations:
            yield to_biorbd_array_output(activation)
