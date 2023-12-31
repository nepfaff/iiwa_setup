from typing import List

from pydrake.all import AbstractValue, Context, InputPort, LeafSystem


class AbstractMultiplexer(LeafSystem):
    """
    A system that multiplexes abstract values from multiple input ports into a single
    vector-valued output port.
    """

    def __init__(self, num_abstract_scalar_inputs: int, abstract_type: type):
        """
        Args:
            num_abstract_scalar_inputs (int): The number of input ports.
            abstract_type (type): The type of the abstract value.
        """
        super().__init__()

        self._input_ports: List[InputPort] = []
        for i in range(num_abstract_scalar_inputs):
            self._input_ports.append(
                self.DeclareAbstractInputPort(
                    f"input_{i}", AbstractValue.Make(abstract_type())
                )
            )
        self._output_port = self.DeclareAbstractOutputPort(
            "output", lambda: AbstractValue.Make(List[abstract_type]), self._multiplex
        )

    def _multiplex(self, context: Context, output: AbstractValue) -> None:
        values = []
        for port in self._input_ports:
            values.append(port.Eval(context))
        output.set_value(values)
