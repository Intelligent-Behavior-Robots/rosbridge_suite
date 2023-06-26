import struct

from numpy import float32, int16, int32, int64, int8
import rosidl_parser.definition  

try:
    from cbor import Tag
except ImportError:
    from rosbridge_library.util.cbor import Tag


LIST_TYPES = [list, tuple]
INT_TYPES = [
    "byte",
    "char",
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
    "int"
]
FLOAT_TYPES = ["float32", "float64"]
STRING_TYPES = ["string"]
BOOL_TYPES = ["bool"]
TIME_TYPES = ["time", "duration"]
BOOL_ARRAY_TYPES = ["bool[]"]
BYTESTREAM_TYPES = ["uint8[]", "char[]"]

# Typed array tags according to <https://tools.ietf.org/html/draft-ietf-cbor-array-tags-00>
# Always encode to little-endian variant, for now.
TAGGED_ARRAY_FORMATS = {
    "uint16[]": (69, "<{}H"),
    "uint32[]": (70, "<{}I"),
    "uint64[]": (71, "<{}Q"),
    "byte[]": (72, "{}b"),
    "int8[]": (72, "{}b"),
    "int16[]": (77, "<{}h"),
    "int32[]": (78, "<{}i"),
    "int64[]": (79, "<{}q"),
    "float32[]": (85, "<{}f"),
    "float64[]": (86, "<{}d"),
}



basic_supported_types = [int8, int16, int32, int64, float32, bool]

basic_supported_types_transform = {
    "double": "float64",
    "float": "float32",
    "int": "int32",
    "array": " "
}

array_supported_types = {
    'b': "int8[]",
    'B': "uint8[]",
    'h': "int16[]",
    'H': "uint16[]",
    'i': "int32[]",
    'I': "uint32[]",
    'l': "int32[]",
    'L': "uint32[]",
    'q': "int64[]",
    'Q': "uint64[]",
    'f': "float32[]",
    'd': "float64[]",
}
    
def extract_cbor_values(msg):
    """Extract a dictionary of CBOR-friendly values from a ROS message.

    Primitive values will be casted to specific Python primitives.

    Typed arrays will be tagged and packed into byte arrays.
    """
    out = {}
    #for slot, slot_type in zip(msg.__slots__, msg._slot_types):
    for slot, slot_type in zip(msg.__slots__, msg.SLOT_TYPES):
        val = getattr(msg, slot)

        try:
            if type(val) is str:
                slot_type = "string"
            elif any ([type(val) is t for t in basic_supported_types]):
                slot_type = type(val).__name__ #"string"
            elif type(val).__name__ in basic_supported_types_transform:
                if type(val).__name__ == "array":
                    slot_type = array_supported_types[val.typecode]
                else:
                    slot_type = basic_supported_types_transform[type(val).__name__]
            # elif(hasattr(slot_type, "value_type") and slot_type.value_type is rosidl_parser.definition.UnboundedSequence):
            #     slot_type = slot_type.value_type.name.lower()
            else:
                try:
                    slot_type = slot_type.value_type.name.lower()
                except:
                    slot_type = slot_type.name.lower()
        except Exception as e:
            print(e)

        slot = str(slot[1:])

        # string
        if slot_type in STRING_TYPES:
            out[slot] = str(val)

        # bool
        elif slot_type in BOOL_TYPES:
            out[slot] = bool(val)

        # integers
        elif slot_type in INT_TYPES:
            out[slot] = int(val)

        # floats
        elif slot_type in FLOAT_TYPES:
            out[slot] = float(val)

        # time/duration
        elif slot_type in TIME_TYPES:
            out[slot] = {
                "secs": int(val.sec),
                "nsecs": int(val.nanosec),
            }

        # byte array
        elif slot_type in BYTESTREAM_TYPES:
            out[slot] = bytes(val)

        # bool array
        elif slot_type in BOOL_ARRAY_TYPES:
            out[slot] = [bool(i) for i in val]

        # numeric arrays
        elif slot_type in TAGGED_ARRAY_FORMATS:
            tag, fmt = TAGGED_ARRAY_FORMATS[slot_type]
            fmt_to_length = fmt.format(len(val))
            packed = struct.pack(fmt_to_length, *val)
            out[slot] = Tag(tag=tag, value=packed)

        # array of messages
        elif type(val) in LIST_TYPES:
            out[slot] = [extract_cbor_values(i) for i in val]

        # message
        else:
            out[slot] = extract_cbor_values(val)

    return out
