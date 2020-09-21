try:
    from _eris._eris import *
except ImportError as ex:
    error_msg = "Failed to import the Eris C-module"
    raise ImportError(error_msg) from ex