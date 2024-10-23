import functools
import select
import socket
import struct
import sys
import signal

import time
import keyboard

import rospy

import numpy as np
from std_msgs.msg import Float32MultiArray

#
# Only load functools in Python version 2.5 or newer.
#
if sys.version_info >= (2, 5):
    import functools

from std_msgs.msg import String

send_data = [[0]*3]*32

def talker():
   pub = rospy.Publisher('MocapSkeleton', String, queue_size=10)
   rospy.init_node('Mocap', anonymous=True)
   rate = rospy.Rate(10) # 10hz
   while not rospy.is_shutdown():
      hello_str = String("[[1.3608, -0.0131, -0.3593], [1.3691, 0.01, -0.0498], [1.3604, 0.0316, 0.258], [1.3798, 0.074, 0.3884], [1.2747, -0.1331, 0.1839], [1.1012, -0.2656, 0.1086], [0.874, -0.2766, 0.0896], [0.7493, -0.2458, 0.0926], [1.3556, 0.1615, 0.153], [1.1495, 0.2204, 0.1081], [0.9461, 0.2612, 0.1213], [0.8847, 0.2567, 0.122], [1.3124, -0.087, -0.334], [0.9972, -0.1952, -0.3109], [0.9521, -0.2096, -0.6592], [0.8373, -0.1785, -0.6794], [1.3555, 0.0595, -0.3415], [1.0667, 0.2201, -0.3475], [0.9928, 0.2173, -0.729], [0.8781, 0.2387, -0.7494], [1.3654, 0.0265, 0.1816], [0.6644, -0.2331, 0.0931], [0.7284, -0.2205, 0.1385], [0.7893, 0.2589, 0.1223], [0.89, 0.273, 0.0722]]")
      #rospy.loginfo(hello_str)
      pub.publish(hello_str)
      rate.sleep()


class Client:
    """
    Implements socket connection and basic binary message protocol for
    client application access to all Motion Service streams. Use the static
    Format methods to convert a binary message into the associated object.
    """

    def __init__(self, host, port):
        """
        Create client socket connection to the Motion Service data stream
        on host:port.
        """
        self.__socket = None
        self.__recv_flags = 0
        self.__send_flags = 0
        self.__description = None
        self.__time_out_second = None
        self.__time_out_second_send = None

        # Set the default host name to the local host.
        if (None == host) or (0 == len(host)):
            host = "127.0.0.1"

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((host, port))

        self.__socket = s

        # Read the first message from the service. It is a
        # string description of the remote service.
        self.__description = self.__receive()

    def __del__(self):
        """
        Destructor. Close the socket connection.
        """
        self.close()

    def close(self):
        """
        Close the socket connection if it exists.
        """
        if None != self.__socket:
            self.__socket.shutdown(2)
            self.__socket.close()

            self.__socket = None

    def isConnected(self):
        """
        Return true if the current connection is active.
        """
        if None != self.__socket:
            return True
        else:
            return False

    def waitForData(self, time_out_second=None):
        """
        Wait until there is incoming data on this client
        connection and then returns True.
        """
        # Default time out is 5 seconds.
        if None == time_out_second:
            time_out_second = 5

        if time_out_second != self.__time_out_second:
            self.__socket.settimeout(time_out_second)
            self.__time_out_second = self.__socket.gettimeout()

        data = self.__receive()
        if None != data:
            return True
        else:
            return False

    def readData(self, time_out_second=None):
        """
        Read a single sample of data from the open connection.

        Returns a single sample of data, or None if the incoming
        data is invalid.
        """
        if None == self.__socket:
            return None

        # Default time out is 1 second.
        if None == time_out_second:
            time_out_second = 1

        if time_out_second != self.__time_out_second:
            self.__socket.settimeout(time_out_second)
            self.__time_out_second = self.__socket.gettimeout()

        return self.__receive()

    def writeData(self, data, time_out_second=None):
        """
        Write a single sample of data to the open connection.

        Returns True iff the message was successfully written
        to the socket. Otherwise returns False.
        """
        if None == self.__socket:
            return False

        if len(data) <= 0:
            return False

        # Default time out is 1 second.
        if None == time_out_second:
            time_out_second = 1

        if time_out_second != self.__time_out_second_send:
            self.__socket.settimeout(time_out_second)
            self.__time_out_second_send = self.__socket.gettimeout()

        return self.__send(data)

    def __receive(self):
        """
        Read a single binary message defined by a length header.
        """
        if None == self.__socket:
            return None

        if False == self.__select_receive():
            return None

        try:
            header_size = struct.calcsize("!I")

            # Single integer network order (=big-endian) message length header.
            header = self.__socket.recv(header_size, self.__recv_flags)
            if header_size != len(header):
                return None

            # Parse the length field, read the raw data field.
            length = struct.unpack("!I", header)[0]

            # Use one or more socket.recv calls to read the data payload.
            data = b""
            while True:
                message = self.__socket.recv(
                    length - len(data),
                    self.__recv_flags)
                if not message or 0 == len(message):
                    return None

                data += message
                if len(data) == length:
                    break
                elif len(data) > length:
                    return None

            return data
        except socket.timeout:
            pass

        return None

    def __send(self, data):
        """
        Write a single binary message defined by a length header.

        Returns true iff the message was successfully written
        to the socket.
        """
        if None == self.__socket:
            return False

        if False == self.__select_send():
            return None

        try:
            # Convert Python 3 strings to byte string.
            if not isinstance(data, bytes):
                data = data.encode("utf-8")

            length = len(data)
            message = struct.pack("!I" + str(length) + "s", length, data)

            send_result = self.__socket.sendall(message, self.__send_flags)
            if None == send_result:
                return True

        except socket.timeout:
            pass

        return False

    def __select_receive(self):
        """
        Use the select function to wait until there is data available to read
        on the internal socket. Returns True iff there is at least one byte
        ready to be read.
        """
        fd = self.__socket.fileno()

        try:
            list, _, _ = select.select(
                [fd], [], [], self.__time_out_second)
            for s in list:
                if fd == s:
                    return True
        except socket.timeout:
            pass

        return False

    def __select_send(self):
        """
        Use the select function to wait until there data can be written to the
        internal socket object.
        """
        fd = self.__socket.fileno()

        try:
            _, list, _ = select.select(
                [], [fd], [], self.__time_out_second_send)
            for s in list:
                if fd == s:
                    return True
        except socket.timeout:
            pass

        return False

#
# END class Client
#


class File:
    """
    Implements a file input stream interface for reading Motion Service
    binary take data files. Provide a simple interface to develop
    external applications that can read Motion take data from disk.

    This class only handles the reading of binary data and conversion
    to arrays of native data types. The Format class implements
    interfaces to the service specific data formats.
    """

    def __init__(self, pathname):
        """
        Open a Motion take data file for reading.

        Set parameter pathname to the file to open as the input stream.
        """
        self.__input = None
        self.__input = open(pathname, "rb")

    def __del__(self):
        """
        Destrucutor. Close the input file stream.
        """
        try:
            self.close()
        except RuntimeError:
            pass

    def close(self):
        """
        Close the input file stream.

        Throws a RuntimeError if the file stream is not open.
        """
        if None != self.__input:
            self.__input.close()
            self.__input = None
        else:
            raise RuntimeError("failed to close input file stream, not open")

    def readData(self, length, real_valued):
        """
        Read a single block of binary data from the current position
        in the input file stream. Convert the block of data into an
        array of length typed elements.

        Integer parameter length defines the required number of typed elements.
        Set boolean parameter real_valued to True if the typed elements are
        real valued, i.e. floats. Set real_valued to false for short integers.
        """
        if None == self.__input:
            return None

        data = None
        if length > 0:
            # Choose the binary format of the array values,
            # "f" == float and "h" == short.
            value_format = "f"
            if False == real_valued:
                value_format = "h"

            element_size = length * struct.calcsize("<" + str(value_format))

            input_buffer = self.__input.read(element_size)
            if element_size == len(input_buffer):
                data = struct.unpack(
                    "<" + str(length) + str(value_format), input_buffer)
            else:
                self.close()

        return data

#
# END class File
#


class Format:
    """
    Motion Service streams send a list of data elements. The static Format
    methods create a map from integer id to array packed data for each
    service specific format.
    """

    class Element:
        """
        Motion Service streams send a list of data elements. The {@link Format}
        functions create a map from integer id to array packed data for each
        service specific format.

        This is an abstract base class to implement a single format specific
        data element. The idea is that a child class implements a format
        specific interface (API) to access individual components of an array of
        packed data.

        For example, the PreviewElement class extends this class
        and provides a PreviewElement.getEuler() method to access
        an array of {x, y, z} Euler angles.
        """

        def __init__(self, data, length, real_valued):
            """
            Initialize element data.
            """
            self.__data = None
            self.__real_valued = None

            if (len(data) == length) or (0 == length):
                self.__data = data
                self.__real_valued = real_valued
            else:
                raise RuntimeError("invalid input data for format element")

        def getData(self, base, length):
            """
            Utility function to copy portions of the packed data array into its
            component elements.

            Parameter base defines starting index to copy data from the
            internal data array.

            Parameter element_length defines the number of data values in this
            component element.

            Returns an array of element_length values, assigned to
            [m_data[i] ... m_data[i+element_length]] if there are valid values
            available or zeros otherwise
            """
            if (None != self.__data) and (base + length <= len(self.__data)):
                return self.__data[base:(base + length)]
            else:
                value = float(0)
                if False == real_valued:
                    value = int(0)
                result = list()
                for i in range(0, length):
                    result.append(value)

        def access(self):
            """
            Direct access to the internal buffer.
            """
            return self.__data

    #
    # END class Element
    #

    class ConfigurableElement(Element):
        """
        The Configurable data services provides access to all data streams in
        a single message. The client selects channels and ordering at the
        start of the connection. The Configurable service sends a map of
        N data elements. Each data element is an array of M single precision
        floating point numbers.
        """

        def __init__(self, data):
            """
            Defines the parameters of this element.
            """
            Format.Element.__init__(self, data, 0, True)

        def value(self, index):
            """
            Get a single channel entry at specified index.
            """
            return self.access()[index]

        def size(self):
            """
            Convenience method. Size accessor.
            """
            return len(self.access())

    #
    # END class ConfigurableElement
    #

    class PreviewElement(Element):
        """
        The Preview service sends a map of N Preview data elements. Use this
        class to wrap a single Preview data element such that we can access
        individual components through a simple API.

        Preview element format:
        id => [global quaternion, local quaternion, local euler, acceleration]
        id => [
            Gqw, Gqx, Gqy, Gqz, Lqw, Lqx, Lqy, Lqz, rx, ry, rz, lax, lay, laz
        ]
        """

        def __init__(self, data):
            """
            Defines the parameters of this element.
            """
            Format.Element.__init__(self, data, 14, True)


        def getInternal(self):
            """
            Get a set of x, y, and z Euler angles that define the
            current orientation. Specified in radians assuming x-y-z
            rotation order. Not necessarily continuous over time, each
            angle lies on the domain [-pi, pi].

            Euler angles are computed on the server side based on the
            current local quaternion orientation.

            Returns a three element array [x, y, z] of Euler angles
            in radians or None if there is no available data
            """
            return self.access()

        def getEuler(self):
            """
            Get a set of x, y, and z Euler angles that define the
            current orientation. Specified in radians assuming x-y-z
            rotation order. Not necessarily continuous over time, each
            angle lies on the domain [-pi, pi].

            Euler angles are computed on the server side based on the
            current local quaternion orientation.

            Returns a three element array [x, y, z] of Euler angles
            in radians or None if there is no available data
            """
            return self.getData(8, 3)

        def getMatrix(self, local):
            """
            Get a 4-by-4 rotation matrix from the current global or local
            quaternion orientation. Specified as a 16 element array in
            row-major order.

            Set parameter local to true get the local orientation, set local
            to false to get the global orientation.
            """
            return Format.quaternion_to_R3_rotation(self.getQuaternion(local))

        def getQuaternion(self, local):
            """
            Get the global or local unit quaternion that defines the current
            orientation.

            @param local set local to true get the local orientation, set local
            to false to get the global orientation

            Returns a four element array [w, x, y, z] that defines
            a unit length quaternion q = w + x*i + y*j + z*k or None
            if there is no available data
            """
            if (local):
                return self.getData(4, 4)
            else:
                return self.getData(0, 4)

        def getAccelerate(self):
            """
            Get x, y, and z of the current estimate of linear acceleration.
            Specified in g.

            Returns a three element array [x, y, z] of of linear acceleration
            channels specified in g or zeros if there is no available data
            """
            return self.getData(11, 3)

    #
    # END class PreviewElement
    #

    class SensorElement(Element):
        """
        The Sensor service provides access to the current un-filtered sensor
        signals in real units. The Sensor service sends a map of N data
        elements. Use this class to wrap a single Sensor data element such
        that we can access individual components through a simple API.

        Sensor element format:
        id => [accelerometer, magnetometer, gyroscope]
        id => [ax, ay, az, mx, my, mz, gx, gy, gz]
        """

        def __init__(self, data):
            """
            Initialize this container identifier with a packed data
            array in the Sensor format.

            Parameter data is a packed array of accelerometer, magnetometer,
            and gyroscope un-filtered signal data.
            """
            Format.Element.__init__(self, data, 9, True)

        def getAccelerometer(self):
            """
            Get a set of x, y, and z values of the current un-filtered
            accelerometer signal. Specified in g where 1 g =
            -9.8 meters/sec^2.

            Domain varies with configuration. Maximum is [-6, 6] g.

            Returns a three element array [x, y, z] of acceleration
            in gs or zeros if there is no available data
            """
            return self.getData(0, 3)

        def getGyroscope(self):
            """
            Get a set of x, y, and z values of the current un-filtered
            gyroscope signal. Specified in degrees/second.

            Valid domain is [-500, 500] degrees/second.

            Returns a three element array [x, y, z] of angular velocity
            in degrees/second or zeros if there is no available data.
            """
            return self.getData(6, 3)

        def getMagnetometer(self):
            """
            Get a set of x, y, and z values of the current un-filtered
            magnetometer signal. Specified in uT (microtesla).

            Domain varies with local magnetic field strength. Expect values
            on [-60, 60] uT (microtesla).

            Returns a three element array [x, y, z] of magnetic field
            strength in uT (microtesla) or zeros if there is no
            available data.
            """
            return self.getData(3, 3)

    #
    # class SensorElement
    #

    class RawElement(Element):
        """
        The Raw service provides access to the current uncalibrated,
        unprocessed sensor signals in signed integer format. The Raw service
        sends a map of N data elements. Use this class to wrap a single Raw
        data element such that we can access individual components through a
        simple API.

        Raw element format:
        id => [accelerometer, magnetometer, gyroscope]
        id => [ax, ay, az, mx, my, mz, gx, gy, gz]

        All sensors output 12-bit integers. Process as 16-bit short integers on
        the server side.
        """

        def __init__(self, data):
            """
            Initialize this container identifier with a packed data
            array in the Raw format.

            Parameter data is a packed array of accelerometer, magnetometer,
            and gyroscope un-filtered signal data.
            """
            Format.Element.__init__(self, data, 9, False)

        def getAccelerometer(self):
            """
            Get a set of x, y, and z values of the current unprocessed
            accelerometer signal.

            Valid domain is [0, 4095].

            Returns a three element array [x, y, z] of raw accelerometer
            output or zeros if there is no available data.
            """
            return self.getData(0, 3)

        def getGyroscope(self):
            """
            Get a set of x, y, and z values of the current unprocessed
            gyroscope signal.

            Valid domain is [0, 4095].

            Returns a three element array [x, y, z] of raw gyroscope
            output or zeros if there is no available data.
            """
            return self.getData(6, 3)

        def getMagnetometer(self):
            """
            Get a set of x, y, and z values of the current unprocessed
            magnetometer signal.

            Valid domain is [0, 4095].

            Returns a three element array [x, y, z] of raw magnetometer
            output or zeros if there is no available data.
            """
            return self.getData(3, 3)

    #
    # class RawElement
    #

    def __Configurable(data):
        """
        Convert a container of binary data into an associative
        container of ConfigurableElement entries.
        """
        return Format.__IdToValueArray(
            data, 0, Format.ConfigurableElement, True)
    Configurable = staticmethod(__Configurable)

    def __Preview(data):
        """
        Convert a container of binary data into an associative
        container of PreviewElement entries.
        """
        return Format.__IdToValueArray(data, 14, Format.PreviewElement, True)
    Preview = staticmethod(__Preview)

    def __Sensor(data):
        """
        Convert a container of binary data into an associative
        container of SensorElement entries.
        """
        return Format.__IdToValueArray(data, 9, Format.SensorElement, True)
    Sensor = staticmethod(__Sensor)

    def __Raw(data):
        """
        Convert a container of binary data into an associative
        container of RawElement entries.
        """
        return Format.__IdToValueArray(data, 9, Format.RawElement, False)
    Raw = staticmethod(__Raw)

    def __IdToValueArray(data, length, factory, real_valued):
        """
        Utility method to convert a packed binary representation of
        an associative container into that container.
        """
        result = {}

        if None == data:
            return result

        # Choose the binary format of the array values,
        # "f" == float and "h" == short.
        value_format = "f"
        if False == real_valued:
            value_format = "h"

        # Prefix "<" for little-endian byte ordering.
        sizeof_key = struct.calcsize("<I")
        sizeof_value = struct.calcsize("<" + str(value_format))

        itr = 0
        while (itr < len(data)) and ((len(data) - itr) > sizeof_key):
            # Read the integer id for this element.
            key = struct.unpack("<I", data[itr:itr + sizeof_key])[0]
            itr += sizeof_key

            # Optionally read the integer length field of the
            # data array.
            element_length = length
            if (0 == element_length) and ((len(data) - itr) > sizeof_key):
                element_length = struct.unpack(
                    "<I", data[itr:itr + sizeof_key])[0]
                itr += sizeof_key

            # Read the array of values for this element.
            sizeof_array = sizeof_value * element_length
            if (element_length > 0) and ((len(data) - itr) >= sizeof_array):
                value = struct.unpack(
                    str(element_length) + value_format,
                    data[itr:itr + sizeof_array])
                itr += sizeof_array

                result[key] = factory(value)

        # If we did not consume all of the input bytes this is an
        # invalid message.
        if len(data) != itr:
            result = {}

        return result

    __IdToValueArray = staticmethod(__IdToValueArray)

    def quaternion_to_R3_rotation(q):
        """
        Ported from the Boost.Quaternion library at:
        http://www.boost.org/libs/math/quaternion/HSO3.hpp

        Parameter q defines a quaternion in the format [w x y z] where
        q = w + x*i + y*j + z*k.

        Returns an array of 16 elements that defines a 4-by-4 rotation
        matrix computed from the input quaternion or the identity matrix
        if the input quaternion has zero length. Matrix is in row-major
        order.
        """
        if 4 != len(q):
            return None

        a = q[0]
        b = q[1]
        c = q[2]
        d = q[3]

        aa = a * a
        ab = a * b
        ac = a * c
        ad = a * d
        bb = b * b
        bc = b * c
        bd = b * d
        cc = c * c
        cd = c * d
        dd = d * d

        norme_carre = aa + bb + cc + dd

        result = list()
        for i in range(0, 4):
            for j in range(0, 4):
                if i == j:
                    result.append(1)
                else:
                    result.append(0)

        if (norme_carre > 1e-6):
            result[0] = (aa + bb - cc - dd) / norme_carre
            result[1] = 2 * (-ad + bc) / norme_carre
            result[2] = 2 * (ac + bd) / norme_carre
            result[4] = 2 * (ad + bc) / norme_carre
            result[5] = (aa - bb + cc - dd) / norme_carre
            result[6] = 2 * (-ab + cd) / norme_carre
            result[8] = 2 * (-ac + bd) / norme_carre
            result[9] = 2 * (ab + cd) / norme_carre
            result[10] = (aa - bb - cc + dd) / norme_carre

        return result

    quaternion_to_R3_rotation = staticmethod(quaternion_to_R3_rotation)
#
# END class Format
#


class LuaConsole:
    """
    Implements the communication protocol with the Motion Service console.
    Send general Lua scripting commands to the Motion Service and receive
    a result code and printed results.
    """

    # The Lua chunk was successfully parsed and executed. The
    # printed results are in the result string.
    Success = 0
    # The Lua chunk failed due to a compile time or execution
    # time error. An error description is in the result string.
    Failure = 1
    # The Lua chunk was incomplete. The Console service is waiting
    # for a complete chunk before it executes.
    # For example, "if x > 1 then" is incomplete since it requires
    # and "end" token to close the "if" control statement.
    Continue = 2

    def __init__(self, client):
        """
        Constructor. Supply the already open client socket connection
        to the Motion Service console..
        """
        self.__client = client

    def send_chunk(self, chunk, time_out_second=None):
        """
        Write a general Lua chunk to the open Console service
        socket and read back the results.
        """
        result_code = self.Failure
        result_string = None

        # Write the Lua chunk.
        if self.__client.writeData(chunk, time_out_second):
            # Read back the response. This is how the Lua Console
            # service works. It will always respond with at least
            # an error code.
            data = self.__client.readData(time_out_second)
            if None != data and len(data) > 0:
                if not isinstance(data, str):
                    data = str(data, "utf-8")

                code = ord(data[0])
                if code >= self.Success and code <= self.Continue:
                    result_code = code
                    if len(data) > 1:
                        result_string = data[1:]

        return result_code, result_string

    def __SendChunk(client, chunk, time_out_second=None):
        """
        A more Python friendly version of the SendChunk method.
        This will throw an exception if there is an error in the
        scripting command. Otherwise, this will only return the
        printed results.
        """
        lua_console = LuaConsole(client)
        result_code, result_string = lua_console.send_chunk(
            chunk, time_out_second)

        if lua_console.Success == result_code:
            return result_string
        elif lua_console.Continue == result_code:
            raise RuntimeError(
                "Lua chunk incomplete: " + str(result_string))
        else:
            raise RuntimeError(
                "Lua command chunk failed: " + str(result_string))

    SendChunk = staticmethod(__SendChunk)

    class Node:
        """
        Utility class to implement a generic scripting interface
        from the Motion Service console (Lua) to Python and vice versa.

        Dispatch named methods with variable length argument
        lists.

        Implements all Lua node.* methods that return a boolean
        and string value pair. Also supports simple string result
        but the client script must handle this correctly.

        Example usage:

        node = LuaConsole.Node(Client("", 32075))
        result, message = node.start()
        if result:
            # Success. We are reading from at least one device.
            pass
        else:
            # Failure. There are no configured devices or the
            # hardware is not available.
            print message
        """
        def __init__(self, client):
            self.__client = client

        def __getattr__(self, name):
            if sys.version_info >= (2, 5):
                return functools.partial(self.__dispatch, name)
            else:
                return None

        def __dispatch(self, name, *arg_list):
            result = self.__string_call(name, *arg_list)

            if result.startswith("true"):
                return True, result[4:]
            elif result.startswith("false"):
                return False, result[5:]
            else:
                return str(result)

        def __string_call(self, name, *arg_list):
            lua_call = "=node.%s(" % name

            # Create a string valued argument list from a variable
            # length list of arguments. Note that this only supports
            # String and Float valued arguments.
            sep = ""
            for item in arg_list:
                if isinstance(item, str):
                    lua_call += "%s%s" % (sep, "".join(
                        ["'", ("\\'").join(i for i in item.split("'")), "'"]))
                else:
                    lua_call += "%s%s" % (sep, float(item))
                sep = ", "

            lua_call += ")"

            return LuaConsole.SendChunk(self.__client, lua_call, 5)

            #
            # END class Node
            #

      #
      #
      # END class LuaConsole
      #

def joint_dictionary(num):
    """ 
    Dictionary of joints in pose [32x3]
    Return:
        joint_dict: Dictionary of pose joints.
    """
    joint_dict = []
    joint_dict.append("Body")               # 0 = SpineBase
    joint_dict.append("Chest")              # 1 = SpineMid
    joint_dict.append("Head")               # 2* = Neck
    joint_dict.append("HeadEnd")            # 3* = Head
    joint_dict.append("Hips")               # 4 = ShoulderLeft
    joint_dict.append("LeftArm")            # 5* = ElbowLeft
    joint_dict.append("LeftFinger")         # 6* = WristLeft
    joint_dict.append("LeftFingerEnd")      # 7 = HandLeft
    joint_dict.append("LeftFoot")           # 8 = ShoulderRight
    joint_dict.append("LeftForearm")        # 9* = ElbowRight
    joint_dict.append("LeftHand")           # 10* = WristRight
    joint_dict.append("LeftHeel")           # 11*= HandRight
    joint_dict.append("LeftLeg")            # 12 = HipLeft
    joint_dict.append("LeftShoulder")       # 13* = KneeLeft
    joint_dict.append("LeftThigh")          # 14* = AnkleLeft
    joint_dict.append("LeftToe")            # 15* = FootLeft
    joint_dict.append("LeftToeEnd")         # 16 = HipRight
    joint_dict.append("Neck")               # 17 = KneeRight
    joint_dict.append("RightArm")           # 18 = AnkleRight
    joint_dict.append("RightFinger")        # 19* = FootRight
    joint_dict.append("RightFingerEnd")     # 20 = SpineShoulder
    joint_dict.append("RightFoot")          # 21 = HandTipLeft
    joint_dict.append("RightForearm")       # 22* = ThumbLeft
    joint_dict.append("RightHand")          # 23* = HandTipRight
    joint_dict.append("RightHeel")          # 24* = ThumbRight
    joint_dict.append("RightLeg")           # 25 =
    joint_dict.append("RightShoulder")      # 26* =
    joint_dict.append("RightThigh")         # 27* =
    joint_dict.append("RightToe")           # 28* =
    joint_dict.append("RightToeEnd")        # 29 =
    joint_dict.append("SpineLow")           # 30 =
    joint_dict.append("SpineMid")           # 31 =
    #joint_dict.append("EXTRA")

    return joint_dict[num]

zero_position = [[0]*4]*32



def set_zero():

    print("set to zero procedure will be started in 4s...")
    time.sleep(4)
    print("Started!")
    data = client.readData()
    data = client.readData()
    print(data, "\n\n")
    conf = Format.Configurable(data)
    # preview = Format.Preview(data)
    # print(conf)
    if client.waitForData():
        if len(conf) > 0:
            item_i = 0
            for key_conf in conf:
                key_list = key_conf - 1
                temp = [0,0,0,0]
                for i in range(conf[key_conf].size()):
                    temp[i] = float("{0:.2f}".format(conf[key_conf].value(i)))
                zero_position[key_list] = temp
    print("Finished !")


def signal_handler(sig, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def main():
    """
    Example usage and test function for the Client, File, Format, and
    LuaConsole classes.
    """

    # Set the default host and port parameters. The SDK is
    # socket bases so any networked Motion Service is available.

    #talker()

    pub = rospy.Publisher('MocapSkeleton', String, queue_size=1)
    joint_pub = rospy.Publisher('MocapSkeleton_msg', Float32MultiArray, queue_size=1)
    rospy.init_node('Mocap', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    global send_data
    Host = "10.0.0.1"   # editted hereeeeeeee! ""
    PortPreview = 32079
    PortConsole = 32075
    PortConfigure = 32076

    #
    # General Lua scripting interface.
    #
    lua_client = Client(Host, PortConsole)
    lua_chunk = \
        "if not node.is_reading() then" \
        "   node.close()" \
        "   node.scan()" \
        "   node.start()" \
        " end" \
        " if node.is_reading() then" \
        "   print('Reading from ' .. node.num_reading() .. ' device(s)')" \
        " else" \
        "   print('Failed to start reading')" \
        " end"

    print(LuaConsole.SendChunk(lua_client, lua_chunk, 5))

    # Scripting language compatibility class. Translate
    # Python calls into Lua calls and send them to the
    # console service.
    if sys.version_info >= (2, 5):
        node = LuaConsole.Node(lua_client)
        print(node.is_reading())



    xml_string = \
        "<configurable stride=\"0\" full=\"0\" inactive=\"1\">" \
        "<all>" \
        "<c/>" \
        "</all>"\
        "</configurable>"

    global client
    client = Client(Host, PortConfigure)

    if client.writeData(xml_string):
        print("Sent active channel definition to Configurable service","\n")
    text = input("set to zero (s) or data streaming (d) ?")
    print(zero_position)
    if text == "s":
        set_zero()
    print(zero_position)

    counter = 0
    if client.waitForData():
        sample_count = 0
        while True:

            data = client.readData()
            conf = Format.Configurable(data)          
            pub.publish(str(send_data) + "#" + str(counter))
            counter = counter + 1
            #rate.sleep()  #delay can cause the Mocap data validation problem
            send_data = [[0]*3]*32
            print("\n",counter,"\n")
            if len(conf) > 0:
                item_i = 0
                for key_conf in conf:
                    key_list = key_conf - 1;
                    line = "data({}) = (".format(joint_dictionary(key_list))
                    t_send_list = [0,0,0,0]
                    for i in range(conf[key_conf].size()):
                        if i > 0:
                            line += ", "
                        t_send_list[i] = float("{0:.3f}".format(conf[key_conf].value(i)-zero_position[key_list][i]))/100
                        line += "{}".format(float("{0:.2f}".format(conf[key_conf].value(i)-zero_position[key_list][i])))
                    line += ")"
                    send_data[key_list] = t_send_list[1:]
                    #send_data[key_list][0] = t_send_list[3]
                    #send_data[key_list][1] = t_send_list[2]
                    #send_data[key_list][2] = t_send_list[1] #first element is mostly zero and NOT related to the position data !
                    #print(line, "  Key = ",key_list)

            else:
                break

            time_stamp = int(rospy.Time.now().to_nsec())
            t_h_2 =int(time_stamp /1000000000000)%1000000
            t_h = int(time_stamp /1000000)%1000000
            t_l = time_stamp %1000000
            send_data.append([t_h_2,t_h, t_l])
            points_arr = np.array(send_data).flatten()
            points_msg = Float32MultiArray()
            points_msg.data = points_arr.tolist()
            joint_pub.publish(points_msg)
            print(points_msg)
            print (time_stamp)
            time_stamp = 0



            sample_count += 1

    else:
        print("No current data available, giving up")
 
    client.close()

# editted : only shows real-time streaming data


if __name__ == "__main__":
    sys.exit(main())

