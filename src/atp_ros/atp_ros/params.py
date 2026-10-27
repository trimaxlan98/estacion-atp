from rcl_interfaces.msg import ParameterDescriptor


def declare_num(node, name, default):
    """Declara un parametro numerico que acepta int o float desde la CLI
    (`el_max:=85` llega como entero y rompe un parametro double)."""
    node.declare_parameter(name, default,
                           ParameterDescriptor(dynamic_typing=True))
