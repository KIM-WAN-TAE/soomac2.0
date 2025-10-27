# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dongsoo_interfaces:srv/DongSooExecutor.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'position'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DongSooExecutor_Request(type):
    """Metaclass of message 'DongSooExecutor_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dongsoo_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dongsoo_interfaces.srv.DongSooExecutor_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__dong_soo_executor__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__dong_soo_executor__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__dong_soo_executor__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__dong_soo_executor__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__dong_soo_executor__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DongSooExecutor_Request(metaclass=Metaclass_DongSooExecutor_Request):
    """Message class 'DongSooExecutor_Request'."""

    __slots__ = [
        '_frame',
        '_position',
        '_look',
        '_time',
        '_wrist',
    ]

    _fields_and_field_types = {
        'frame': 'string',
        'position': 'float[4]',
        'look': 'string',
        'time': 'float',
        'wrist': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.frame = kwargs.get('frame', str())
        if 'position' not in kwargs:
            self.position = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.position = kwargs.get('position')
        self.look = kwargs.get('look', str())
        self.time = kwargs.get('time', float())
        self.wrist = kwargs.get('wrist', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.frame != other.frame:
            return False
        if any(self.position != other.position):
            return False
        if self.look != other.look:
            return False
        if self.time != other.time:
            return False
        if self.wrist != other.wrist:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def frame(self):
        """Message field 'frame'."""
        return self._frame

    @frame.setter
    def frame(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'frame' field must be of type 'str'"
        self._frame = value

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'position' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 4, \
                "The 'position' numpy.ndarray() must have a size of 4"
            self._position = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'position' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._position = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def look(self):
        """Message field 'look'."""
        return self._look

    @look.setter
    def look(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'look' field must be of type 'str'"
        self._look = value

    @builtins.property
    def time(self):
        """Message field 'time'."""
        return self._time

    @time.setter
    def time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'time' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'time' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._time = value

    @builtins.property
    def wrist(self):
        """Message field 'wrist'."""
        return self._wrist

    @wrist.setter
    def wrist(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'wrist' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'wrist' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._wrist = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_DongSooExecutor_Response(type):
    """Metaclass of message 'DongSooExecutor_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dongsoo_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dongsoo_interfaces.srv.DongSooExecutor_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__dong_soo_executor__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__dong_soo_executor__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__dong_soo_executor__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__dong_soo_executor__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__dong_soo_executor__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DongSooExecutor_Response(metaclass=Metaclass_DongSooExecutor_Response):
    """Message class 'DongSooExecutor_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_DongSooExecutor(type):
    """Metaclass of service 'DongSooExecutor'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dongsoo_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dongsoo_interfaces.srv.DongSooExecutor')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__dong_soo_executor

            from dongsoo_interfaces.srv import _dong_soo_executor
            if _dong_soo_executor.Metaclass_DongSooExecutor_Request._TYPE_SUPPORT is None:
                _dong_soo_executor.Metaclass_DongSooExecutor_Request.__import_type_support__()
            if _dong_soo_executor.Metaclass_DongSooExecutor_Response._TYPE_SUPPORT is None:
                _dong_soo_executor.Metaclass_DongSooExecutor_Response.__import_type_support__()


class DongSooExecutor(metaclass=Metaclass_DongSooExecutor):
    from dongsoo_interfaces.srv._dong_soo_executor import DongSooExecutor_Request as Request
    from dongsoo_interfaces.srv._dong_soo_executor import DongSooExecutor_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
