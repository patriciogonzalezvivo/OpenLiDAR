"""NexStar hand controller command API.

Contains a single class NexStar which provides an API for the NexStar hand controller command set.
The serial command protocol is documented here:
http://www.nexstarsite.com/download/manuals/NexStarCommunicationProtocolV1.2.zip
"""


import datetime
import calendar
import serial


__all__ = ['NexStar']

# pylint: disable=too-many-public-methods
class NexStar:
    """Implements the serial commands used by NexStar telescope mount hand controllers."""

    class ResponseException(Exception):
        """Raised on bad command responses from NexStar.

        Attributes:
            response (bytes): The byte array which was received but was deemed invalid. This can be
                inspected to see what the (possibly partial) bad response looks like. Do not expect
                this buffer to necessarily match the expected response length; it could be shorter
                or longer. Note also that the terminating '#' character is not included.
        """
        def __init__(self, response, *args, **kwargs):
            self.response = response
            super().__init__(*args, **kwargs)

    class ReadTimeoutException(Exception):
        """Raised when read from NexStar times out."""

    def __init__(self, device, read_timeout=3.5):
        """Constructs a NexStar object.

        Args:
            device (str): The path to the serial device connected to the NexStar hand controller.
                For example, '/dev/ttyUSB0'.
            read_timeout (float): Timeout in seconds for reads on the serial device.
        """
        self.serial = serial.Serial(device, baudrate=9600, timeout=read_timeout)

    def __del__(self):
        """Destructs a NexStar object.

        Any GOTO in progress will be cancelled and any active slewing will be stopped.
        """
        self.cancel_goto()
        self.slew_fixed('az', 0)
        self.slew_fixed('alt', 0)

    def _send_command(self, command, response_len=None):
        """Sends a command to the NexStar hand controller and reads back the response.

        Args:
            command: A byte array containing the ASCII command to send.
            response_len: An integer giving the expected length of the response to this command,
                not counting the terminating '#' character. If set, a ResponseException will be
                raised if the actual response string does not have this length. If None, no length
                validation is performed.

        Returns:
            A byte array containing the response from the hand controller, excluding the
            termination character.

        Raises:
            ReadTimeoutException: When a timeout occurs during the attempt to read from the serial
                device.
            ResponseException: When the response length does not match the value of the
                response_len argument.
        """

        # Eliminate any stale data sitting in the read buffer which could be left over from prior
        # command responses.
        self.serial.read(self.serial.in_waiting)

        self.serial.write(command)

        response = self.serial.read_until(terminator=b'#')
        if response[-1:] != b'#':
            raise NexStar.ReadTimeoutException()

        # strip off the '#' terminator
        response = response[:-1]
        assert b'#' not in response, 'Unexpected terminator found in response'

        if response_len is not None:
            if len(response) != response_len:
                raise NexStar.ResponseException(
                    response,
                    'Expected response length {:d} but got {:d} instead.'.format(response_len,
                                                                                 len(response)))

        return response

    @staticmethod
    def _precise_to_degrees(precise):
        """Decodes angle from a bytearray in NexStar precise format to a float in degrees.

        Args:
            precise (bytearray): An angle encoded as a string in the precise angle format. See the
                NexStar command reference for details on this encoding.

        Returns:
            float: Angle in degrees. Value will be in range [0,360).
        """
        return int(precise, 16) / 2.**32 * 360.

    @staticmethod
    def _degrees_to_precise(degrees):
        """Encodes angular values as a bytearray in NexStar precise angle format.

        Args:
            degrees (float): The angle to encode in degrees. No restrictions are placed on the
                range of values allowed. Both positive and negative angles are supported.

        Returns:
            bytes: The angle encoded in NexStar precise format. See the NexStar command reference
            for details on this encoding.

        """
        return b'%08X' % round((degrees % 360.) / 360. * 2.**32)

    def _get_position(self, command_char):
        """Generic "get position" command helper function.

        Args:
            command_char (bytearray): A single character, either 'e' for RA/DEC or 'z' for
                AZ/ALT. Only the precise version of these commands are supported, therefore 'E'
                and 'Z' are not allowed.

        Returns:
            tuple of floats: A pair of angles in degrees: (ra, dec) or (az, alt) depending on the
            command_char argument.

        """
        assert command_char in [b'e', b'z']
        response = self._send_command(command_char, 17)
        return (self._precise_to_degrees(response[0: 8]),
                self._precise_to_degrees(response[9:17]))

    def get_azalt(self):
        """Get current mount position in horizontal (azimuth/altitude) coordinates.

        Note that if an alignment has not been performed the horizontal coordinates reported by the
        hand controller will be relative to where the mount was pointed when powered on.

        Returns:
            tuple of floats: A pair of angles (azimuth, altitude) in degrees. Azimuth range is
            [0,360) and altitude range is [-180,180).

        """
        # pylint: disable=invalid-name
        (az, alt) = self._get_position(b'z')
        # adjust range of altitude from [0,360) to [-180,180)
        alt = (alt + 180.0) % 360.0 - 180.0
        return (az, alt)

    def get_radec(self):
        """Get current mount position in equatorial (right ascension/declination) coordinates.

        Note that the equatorial position reported by the hand controller will not be meaningful
        until an alignment has been performed.

        Returns:
            tuple of floats: A pair of angles (right ascension, declination) in degrees.
        """
        return self._get_position(b'e')

    def _goto(self, command_char, values):
        """Generic "goto" command helper function.

        Args:
            command_char (bytearray): A single character; 'b' for RA/DEC goto, 'r' for AZ/ALT goto,
                or 's' for sync. Only the precise version of these commands are supported,
                therefore 'B', 'R', and 'S' are not allowed.
            values (tuple of floats): A pair of angles in degrees: (ra, dec) or (az, alt) depending
                on the command_char argument.

        """
        assert command_char in [b'b', b'r', b's']
        command = (command_char
                   + self._degrees_to_precise(values[0])
                   + b','
                   + self._degrees_to_precise(values[1]))
        self._send_command(command)

    def goto_azalt(self, az, alt):
        """Go to a position in horizontal (azimuth/altitude) coordinates.

        Args:
            az (float): Azimuth angle in degrees.
            alt (float): Altitude angle in degrees.
        """
        # pylint: disable=invalid-name
        self._goto(b'b', (az, alt))

    def goto_radec(self, ra, dec):
        """Go to a position in equatorial (right ascension/declination) coordinates.

        Args:
            ra (float): Right ascension angle in degrees.
            dec (float): Declination angle in degrees.
        """
        # pylint: disable=invalid-name
        self._goto(b'r', (ra, dec))

    def sync(self, ra, dec):
        """Sync mount to a position in equatorial (right ascension/declination) coordinates.

        This command informs the hand controller that the mount is currently pointed at the
        provided coordinates to improve accuracy of future nearby goto commands. See NexStar
        command reference for details.

        Args:
            ra (float): Right ascension angle in degrees.
            dec (float): Declination angle in degrees.
        """
        # pylint: disable=invalid-name
        self._goto(b's', (ra, dec))

    def get_tracking_mode(self):
        """Get the current tracking mode.

        Returns:
            int: The current tracking mode where
            0 = Off
            1 = Alt/Az
            2 = EQ North
            3 = EQ South
        """
        response = self._send_command(b't', 1)
        return response[0]

    def set_tracking_mode(self, mode):
        """Set the tracking mode.

        This command has no effect before an alignment is performed. Prior to alignment, the
        tracking mode will always be 0 (off).

        Args:
            mode (int): The mode to set. Must be one of the following values:
                0 = Off
                1 = Alt/Az
                2 = EQ North
                3 = EQ South
        """
        assert mode in range(0, 4)
        command = b'T' + bytes([mode])
        self._send_command(command)

    def slew_var(self, axis, rate):
        """Variable-rate slew command.

        Variable-rate simply means that the angular rate can be specified precisely in arcseconds
        per second, in contrast to the nine fixed rates available on the hand controller keypad.

        Args:
            axis (str): The mount axis to command. Use 'az' or 'alt' for AZ/ALT mounts. Use 'ra' or
                'dec' for equatorial mounts.
            rate (float): The desired slew rate in arcseconds per second. Value may be positive or
                negative. The maximum rate may be mount dependent. The maximum advertised rate for
                the NexStar 130SLT is 3 deg/s. However the maximum commandable rate for the same
                model was found by experimentation to be 16319 arcseconds per second or ~4.5 deg/s.
        """
        assert axis in ['az', 'alt', 'ra', 'dec']
        command = b'P' + bytes([
            3,  # variable rate slew
            16 if axis in ['az', 'ra']  else 17,  # axis
            7 if rate < 0 else 6,  # sign of rate
            (int(abs(rate)) * 4) // 256,  # upper byte of rate magnitude
            (int(abs(rate)) * 4) % 256,  # lower byte of rate magnitude
            0,
            0,
        ])
        self._send_command(command)

    def slew_fixed(self, axis, rate):
        """Fixed-rate slew command.

        Fixed-rate means that only the nine rates supported on the hand controller keypad are
        available.

        Args:
            axis (str): The mount axis to command. Use 'az' or 'alt' for AZ/ALT mounts. Use 'ra' or
                'dec' for equatorial mounts.
            rate (int): The desired slew rate from -9 to +9. Use value 0 to stop motion.
        """
        assert axis in ['az', 'alt']
        assert -9 <= rate <= 9, 'fixed slew rate out of range'
        command = b'P' + bytes([
            2,  # fixed rate slew
            16 if axis in ['az', 'ra'] else 17,  # axis
            37 if rate < 0 else 36,  # sign of rate
            int(abs(rate)),  # rate magnitude
            0,
            0,
            0,
        ])
        self._send_command(command)

    def get_location(self):
        """Get the mount location on Earth in geographic (latitude/longitude) coordinates.

        Returns:
            tuple of floats: A pair of angles (latitude, longitude) in signed degrees format.
        """
        response = self._send_command(b'w', 8)
        lat_deg = response[0]
        lat_min = response[1]
        lat_sec = response[2]
        lat_north = (response[3] == 0)
        lat = lat_deg + lat_min / 60.0 + lat_sec / 3600.0
        if not lat_north:
            lat = -lat
        lon_deg = response[4]
        lon_min = response[5]
        lon_sec = response[6]
        lon_east = (response[7] == 0)
        lon = lon_deg + lon_min / 60.0 + lon_sec / 3600.0
        if not lon_east:
            lon = -lon
        return (lat, lon)

    def set_location(self, lat, lon):
        """Set the mount location on Earth in geographic (latitude/longitude) coordinates.

        Args:
            lat (float): Latitude in signed degrees format.
            lon (float): Longitude in signed degrees format.
        """
        lat_deg = int(abs(lat))
        lat_min = int((abs(lat) - lat_deg) * 60.0)
        lat_sec = int((abs(lat) - lat_deg - lat_min / 60.0) * 3600.0)
        lon_deg = int(abs(lon))
        lon_min = int((abs(lon) - lon_deg) * 60.0)
        lon_sec = int((abs(lon) - lon_deg - lon_min / 60.0) * 3600.0)
        command = b'W' + bytes([
            lat_deg,
            lat_min,
            lat_sec,
            lat < 0,
            lon_deg,
            lon_min,
            lon_sec,
            lon < 0,
        ])
        self._send_command(command)

    def get_time(self):
        """Get the current time from the hand controller in seconds since the Unix epoch.

        Timezone information and daylight savings time are not currently supported. If the hand
        controller is set to a non-zero UTC offset or if daylight savings time is enabled the
        Unix timestamp returned by this function will most likely be incorrect.

        Returns:
            int: A Unix timestamp (seconds since 1 Jan 1970 in UTC minus leap seconds)
        """
        response = self._send_command(b'h', 8)
        hand_controller_time = datetime.datetime(
            response[5] + 2000, # year
            response[3],        # month
            response[4],        # day
            response[0],        # hour
            response[1],        # minute
            response[2],        # second
            0,                  # microseconds
        )
        return calendar.timegm(hand_controller_time.timetuple())

    def set_time(self, timestamp=None):
        """Set the time on the hand controller.

        Timezone information and daylight savings time are not currently supported. When this
        method is called, the UTC offset will be set to 0 and daylight savings time will be
        disabled. This effectively means that the hand controller clock will be set to UTC.

        Args:
            timestamp (int): A Unix timestamp (seconds since 1 Jan 1970 in UTC minus leap seconds).
                If omitted, the time will be obtained from the clock of the machine running this
                Python program.
        """
        if timestamp is not None:
            utc_time = datetime.datetime.utcfromtimestamp(timestamp)
        else:
            utc_time = datetime.datetime.utcnow()

        command = b'H' + bytes([
            utc_time.hour,
            utc_time.minute,
            utc_time.second,
            utc_time.month,
            utc_time.day,
            utc_time.year - 2000,
            0,  # UTC offset
            0,  # disable daylight savings
        ])
        self._send_command(command)

    def get_gps_lock_status(self):
        """Get the status of GPS lock

        Returns:
            bool: True if GPS is linked (locked?), false if GPS is not linked (no lock?)
        """
        command = b'P' + bytes([
            1,
            176,  # GPS Device ID?
            55,  # Device register?
            0,
            0,
            0,
            1,  # GPS response bytes?
        ])

        response = self._send_command(command, 1)

        return bool(response[0])

    def get_gps_location(self):
        """Get the GPS position in geographic (latitude/longitude) coordinates.

        Returns:
            tuple of floats: A pair of angles (latitude, longitude) in signed degrees format.
        """
        [x_var, y_var, z_var] = self._send_command(b'P' + bytes([1, 176, 1, 0, 0, 0, 3]), 3)
        lat = ((x_var * 65536) + (y_var * 256) + z_var) / (2. ** 24) * 360

        [x_var, y_var, z_var] = self._send_command(b'P' + bytes([1, 176, 2, 0, 0, 0, 3]), 3)
        lon = ((x_var * 65536) + (y_var * 256) + z_var) / (2. ** 24) * 360

        return (lat, lon)

    def get_gps_time(self):
        """Get the current GPS time in seconds since the Unix epoch.

        Returns:
            int: A Unix timestamp (seconds since 1 Jan 1970 in UTC minus leap seconds)
        """
        [x_var, y_var] = self._send_command(b'P' + bytes([1, 176, 4, 0, 0, 0, 2]), 2)
        year = (x_var * 256) + y_var

        [month, day] = self._send_command(b'P' + bytes([1, 176, 3, 0, 0, 0, 2]), 2)

        [hour, minute, second] = self._send_command(b'P' + bytes([1, 176, 51, 0, 0, 0, 3]), 3)

        hand_controller_time = datetime.datetime(
            year,  # year
            month,  # month
            day,  # day
            hour,  # hour
            minute,  # minute
            second,  # second
            0,  # microseconds
        )

        return calendar.timegm(hand_controller_time.timetuple())

    def get_version(self):
        """Get hand controller firmware version.

        Returns:
            tuple of ints: Firmware version number as (major, minor) tuple.
        """
        response = self._send_command(b'V', 2)
        return (response[0], response[1])

    def get_model(self):
        """Get mount model.

        Returns:
            int: Mount model encoded as an integer. According to V1.2 of the protocol documentation
            this can be decoded using the following table:
            1 = GPS Series
            3 = i-Series
            4 = i-Series SE
            5 = CGE
            6 = Advanced GT
            7 = SLT
            9 = CPC
            10 = GT
            11 = 4/5 SE
            12 = 6/8 SE
        """
        response = self._send_command(b'm', 1)
        return response[0]

    def get_device_version(self, dev):
        """Get device firmware version.

        Args:
            dev (int): A value from the following table (from protocol documentation):
                16 = AZM/RA Motor
                17 = ALT/DEC Motor
                176 = GPS Unit
                178 = RTC (CGE only)

        Returns:
            tuple of ints: Device firmware version number as (major, minor) tuple.

        Raises:
            ResponseException: When the device does not respond. In this case the response length
                will be 3 bytes rather than the usual 2, but the actual content of the response is
                garbage. See the Developer Notes section of the serial protocol documentation for
                details.
        """
        command = b'P' + bytes([
            1,
            dev,
            254,
            0,
            0,
            0,
            2,
        ])
        response = self._send_command(command, 2)
        return (response[0], response[1])

    def echo(self, echo_val):
        """Send an echo command.

        This command can be used to test the integrity of the serial interface. A single byte
        value is sent with the command that the hand controller will echo back.

        Args:
            echo_val (int): An integer in range(0, 256) to be sent.

        Returns:
            int: The value in the response from the hand controller, which will always be the same
            as echo_val. If the response does not match echo_val an exception is raised.

        Raises:
            ValueError: When echo_val is not in range(0, 256).
            ResponseException: When the response from the hand controller does not contain
                echo_val.
        """
        command = b'K' + bytes([echo_val])
        response = self._send_command(command, 1)
        if response[0] != echo_val:
            raise NexStar.ResponseException(response, 'Echo response does not match what was sent')
        return response[0]

    def alignment_complete(self):
        """Checks if alignment has been completed.

        Returns:
            bool: True if mount is aligned, False otherwise.
        """
        response = self._send_command(b'J', 1)
        return response[0] == 1

    def goto_in_progress(self):
        """Check if a GOTO command is in progress.

        Note that this cannot be used to detect if the mount is in motion because the return value
        will only be True when the mount is performing a GOTO operation. If the mount is slewing
        for any other reason, such as when slew_var or slew_fixed are called, this method will
        return False.

        Returns:
            bool: True if a GOTO is in progress, False otherwise.
        """
        response = self._send_command(b'L', 1)
        return response == b'1'

    def cancel_goto(self):
        """Cancels any GOTO operation that is in progress.

        Has no effect when no GOTO operation is in progress.
        """
        self._send_command(b'M')
