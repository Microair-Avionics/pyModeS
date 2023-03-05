"""ADS-B module.

The ADS-B module also imports functions from the following modules:

- pyModeS.decoder.bds.bds05: ``airborne_position()``, ``airborne_position_with_ref()``, ``altitude()``
- pyModeS.decoder.bds.bds06: ``surface_position()``, ``surface_position_with_ref()``, ``surface_velocity()``
- pyModeS.decoder.bds.bds08: ``category()``, ``callsign()``
- pyModeS.decoder.bds.bds09: ``airborne_velocity()``, ``altitude_diff()``

"""

import pyModeS as pms

from pyModeS import common

from pyModeS.decoder import uncertainty

# from pyModeS.decoder.bds import bds05, bds06, bds09
from pyModeS.decoder.bds.bds05 import (
    airborne_position,
    airborne_position_with_ref,
    altitude as altitude05,
)
from pyModeS.decoder.bds.bds06 import (
    surface_position,
    surface_position_with_ref,
    surface_velocity,
)
from pyModeS.decoder.bds.bds08 import category, callsign
from pyModeS.decoder.bds.bds09 import airborne_velocity, altitude_diff
from pyModeS.decoder.bds.bds61 import is_emergency, emergency_state, emergency_squawk
from pyModeS.decoder.bds.bds62 import (
    selected_altitude,
    selected_heading,
    target_altitude,
    target_angle,
    tcas_operational,
    tcas_ra,
    baro_pressure_setting,
    vertical_mode,
    horizontal_mode,
    vnav_mode,
    lnav_mode,
    autopilot,
    altitude_hold_mode,
    approach_mode,
    emergency_status
)


def df(msg):
    return common.df(msg)


def icao(msg):
    return common.icao(msg)


def typecode(msg):
    return common.typecode(msg)


def position(msg0, msg1, t0, t1, lat_ref=None, lon_ref=None):
    """Decode surface or airborne position from a pair of even and odd
    position messages.

    Note, that to decode surface position using the position message pair,
    the reference position has to be provided.

    Args:
        msg0 (string): even message (28 hexdigits)
        msg1 (string): odd message (28 hexdigits)
        t0 (int): timestamps for the even message
        t1 (int): timestamps for the odd message
        lat_ref (float): latitude of reference position
        lon_ref (float): longitude of reference position

    Returns:
        (float, float): (latitude, longitude) of the aircraft

    """
    tc0 = typecode(msg0)
    tc1 = typecode(msg1)

    if 5 <= tc0 <= 8 and 5 <= tc1 <= 8:
        if lat_ref is None or lon_ref is None:
            raise RuntimeError(
                "Surface position encountered, a reference position"
                " lat/lon required. Location of receiver can be used."
            )
        else:
            return surface_position(msg0, msg1, t0, t1, lat_ref, lon_ref)

    elif 9 <= tc0 <= 18 and 9 <= tc1 <= 18:
        # Airborne position with barometric height
        return airborne_position(msg0, msg1, t0, t1)

    elif 20 <= tc0 <= 22 and 20 <= tc1 <= 22:
        # Airborne position with GNSS height
        return airborne_position(msg0, msg1, t0, t1)

    else:
        #raise RuntimeError("Incorrect or inconsistent message types")
        return 0,0


def position_with_ref(msg, lat_ref, lon_ref):
    """Decode position with only one message.

    A reference position is required, which can be previously
    calculated location, ground station, or airport location.
    The function works with both airborne and surface position messages.
    The reference position shall be within 180NM (airborne) or 45NM (surface)
    of the true position.

    Args:
        msg (str): even message (28 hexdigits)
        lat_ref: previous known latitude
        lon_ref: previous known longitude

    Returns:
        (float, float): (latitude, longitude) of the aircraft
    """

    tc = typecode(msg)

    if 5 <= tc <= 8:
        return surface_position_with_ref(msg, lat_ref, lon_ref)

    elif 9 <= tc <= 18 or 20 <= tc <= 22:
        return airborne_position_with_ref(msg, lat_ref, lon_ref)

    else:
        #raise RuntimeError("incorrect or inconsistent message types")
        return 0,0


def altitude(msg):
    """Decode aircraft altitude.

    Args:
        msg (str): 28 hexdigits string

    Returns:
        int: altitude in feet

    """
    tc = typecode(msg)

    if 0 < tc < 5 or tc == 19 or tc > 22:
        raise RuntimeError("%s: Not a position message" % msg)

    elif tc >= 5 and tc <= 8:
        # surface position, altitude 0
        return 0

    else:
        # airborn position
        return altitude05(msg)


def velocity(msg, source=False):
    """Calculate the speed, heading, and vertical rate (handles both airborne or surface message).

    Args:
        msg (str): 28 hexdigits string
        source (boolean): Include direction and vertical rate sources in return. Default to False.
            If set to True, the function will return six values instead of four.

    Returns:
        (int, float, int, string, [string], [string]): Four or six parameters, including:
            - Speed (kt)
            - Angle (degree), either ground track or heading
            - Vertical rate (ft/min)
            - Speed type ('GS' for ground speed, 'AS' for airspeed)
            - [Optional] Direction source ('TRUE_NORTH' or 'MAGNETIC_NORTH')
            - [Optional] Vertical rate source ('BARO' or 'GNSS')

        For surface messages, vertical rate and its respective sources are set to None.

    """
    if 5 <= typecode(msg) <= 8:
        return surface_velocity(msg, source)

    elif typecode(msg) == 19:
        return airborne_velocity(msg, source)

    else:
        raise RuntimeError(
            "incorrect or inconsistent message types, expecting 4<TC<9 or TC=19"
        )


def speed_heading(msg):
    """Get speed and ground track (or heading) from the velocity message
    (handles both airborne or surface message)

    Args:
        msg (str): 28 hexdigits string

    Returns:
        (int, float): speed (kt), ground track or heading (degree)
    """
    spd, trk_or_hdg, rocd, tag = velocity(msg)
    return spd, trk_or_hdg


def emergency(msg):
    """Get emergency status from the Aircraft Status message

    Args:
        msg (str): 28 hexdigits string

    Returns:
        (int): status
    """

    tc = typecode(msg)

    if tc != 28:
        raise RuntimeError(
            "%s: Not a aircraft status message, expecting TC = 28" % msg
        )

    msgbin = common.hex2bin(msg)
    return common.bin2int(msgbin[40:43])

def modeAcode(msg):
    """Get mode A code from the Aircraft Status message

    Args:
        msg (str): 28 hexdigits string

    Returns:
        (int): status
    """

    tc = typecode(msg)

    if tc != 28:
        raise RuntimeError(
            "%s: Not a aircraft status message, expecting TC = 28" % msg
        )

    msgbin = common.hex2bin(msg)

    a = int(msgbin[44]) + int(msgbin[46]) * 2 + int(msgbin[48]) * 4
    b = int(msgbin[50]) + int(msgbin[52]) * 2 + int(msgbin[54]) * 4
    c = int(msgbin[43]) + int(msgbin[45]) * 2 + int(msgbin[47]) * 4
    d = int(msgbin[51]) + int(msgbin[53]) * 2 + int(msgbin[55]) * 4

    #print(a,b,c,d)
    
    return int(a*1000 + b*100 + c*10 + d)

def oe_flag(msg):
    """Check the odd/even flag. Bit 54, 0 for even, 1 for odd.
    Args:
        msg (str): 28 hexdigits string
    Returns:
        int: 0 or 1, for even or odd frame
    """
    msgbin = common.hex2bin(msg)
    return int(msgbin[53])


def version(msg):
    """ADS-B Version

    Args:
        msg (str): 28 hexdigits string, TC = 31

    Returns:
        int: version number
    """
    tc = typecode(msg)

    if tc != 31:
        raise RuntimeError(
            "%s: Not a status operation message, expecting TC = 31" % msg
        )

    msgbin = common.hex2bin(msg)
    version = common.bin2int(msgbin[72:75])

    return version


def nuc_p(msg):
    """Calculate NUCp, Navigation Uncertainty Category - Position (ADS-B version 1)

    Args:
        msg (str): 28 hexdigits string,

    Returns:
        int: Horizontal Protection Limit
        int: 95% Containment Radius - Horizontal (meters)
        int: 95% Containment Radius - Vertical (meters)

    """
    tc = typecode(msg)

    if typecode(msg) < 5 or typecode(msg) > 22:
        raise RuntimeError(
            "%s: Not a surface position message (5<TC<8), \
            airborne position message (8<TC<19), \
            or airborne position with GNSS height (20<TC<22)"
            % msg
        )

    try:
        NUCp = uncertainty.TC_NUCp_lookup[tc]
        HPL = uncertainty.NUCp[NUCp]["HPL"]
        RCu = uncertainty.NUCp[NUCp]["RCu"]
        RCv = uncertainty.NUCp[NUCp]["RCv"]
    except KeyError:
        HPL, RCu, RCv = uncertainty.NA, uncertainty.NA, uncertainty.NA

    if tc in [20, 21]:
        RCv = uncertainty.NA

    return HPL, RCu, RCv


def nuc_v(msg):
    """Calculate NUCv, Navigation Uncertainty Category - Velocity (ADS-B version 1)

    Args:
        msg (str): 28 hexdigits string,

    Returns:
        int or string: 95% Horizontal Velocity Error
        int or string: 95% Vertical Velocity Error
    """
    tc = typecode(msg)

    if tc != 19:
        raise RuntimeError(
            "%s: Not an airborne velocity message, expecting TC = 19" % msg
        )

    msgbin = common.hex2bin(msg)
    NUCv = common.bin2int(msgbin[42:45])

    try:
        HVE = uncertainty.NUCv[NUCv]["HVE"]
        VVE = uncertainty.NUCv[NUCv]["VVE"]
    except KeyError:
        HVE, VVE = uncertainty.NA, uncertainty.NA

    return HVE, VVE


def nic_v1(msg, NICs):
    """Calculate NIC, navigation integrity category, for ADS-B version 1

    Args:
        msg (str): 28 hexdigits string
        NICs (int or string): NIC supplement

    Returns:
        int or string: Horizontal Radius of Containment
        int or string: Vertical Protection Limit
    """
    if typecode(msg) < 5 or typecode(msg) > 22:
        raise RuntimeError(
            "%s: Not a surface position message (5<TC<8), \
            airborne position message (8<TC<19), \
            or airborne position with GNSS height (20<TC<22)"
            % msg
        )

    tc = typecode(msg)
    NIC = uncertainty.TC_NICv1_lookup[tc]

    if isinstance(NIC, dict):
        NIC = NIC[NICs]

    try:
        Rc = uncertainty.NICv1[NIC][NICs]["Rc"]
        VPL = uncertainty.NICv1[NIC][NICs]["VPL"]
    except KeyError:
        Rc, VPL = uncertainty.NA, uncertainty.NA

    return Rc, VPL


def nic_v2(msg, NICa, NICbc):
    """Calculate NIC, navigation integrity category, for ADS-B version 2

    Args:
        msg (str): 28 hexdigits string
        NICa (int or string): NIC supplement - A
        NICbc (int or string): NIC supplement - B or C

    Returns:
        int or string: Horizontal Radius of Containment
    """
    if typecode(msg) < 5 or typecode(msg) > 22:
        raise RuntimeError(
            "%s: Not a surface position message (5<TC<8), \
            airborne position message (8<TC<19), \
            or airborne position with GNSS height (20<TC<22)"
            % msg
        )

    tc = typecode(msg)
    NIC = uncertainty.TC_NICv2_lookup[tc]

    if 20 <= tc <= 22:
        NICs = 0
    else:
        NICs = NICa * 2 + NICbc

    try:
        if isinstance(NIC, dict):
            NIC = NIC[NICs]

        Rc = uncertainty.NICv2[NIC][NICs]["Rc"]
    except KeyError:
        Rc = uncertainty.NA

    return Rc


def nic_s(msg):
    """Obtain NIC supplement bit, TC=31 message

    Args:
        msg (str): 28 hexdigits string

    Returns:
        int: NICs number (0 or 1)
    """
    tc = typecode(msg)

    if tc != 31:
        raise RuntimeError(
            "%s: Not a status operation message, expecting TC = 31" % msg
        )

    msgbin = common.hex2bin(msg)
    nic_s = int(msgbin[75])

    return nic_s


def nic_a_c(msg):
    """Obtain NICa/c, navigation integrity category supplements a and c

    Args:
        msg (str): 28 hexdigits string

    Returns:
        (int, int/string): NICa and NICc number (0 or 1)
    """
    tc = typecode(msg)

    if tc != 31:
        raise RuntimeError(
            "%s: Not a status operation message, expecting TC = 31" % msg
        )

    msgbin = common.hex2bin(msg)
    nic_a = int(msgbin[75])
    if int(msgbin[37:40]) == 1:
        nic_c = int(msgbin[51])
        return nic_a, nic_c
    else:
        return nic_a, '--'


def nic_b(msg):
    """Obtain NICb, navigation integrity category supplement-b

    Args:
        msg (str): 28 hexdigits string

    Returns:
        int: NICb number (0 or 1)
    """
    tc = typecode(msg)

    if tc < 9 or tc > 18:
        raise RuntimeError(
            "%s: Not a airborne position message, expecting 8<TC<19" % msg
        )

    msgbin = common.hex2bin(msg)
    nic_b = int(msgbin[39])

    return nic_b


def nac_p(msg):
    """Calculate NACp, Navigation Accuracy Category - Position

    Args:
        msg (str): 28 hexdigits string, TC = 29 or 31

    Returns:
        int or string: 95% horizontal accuracy bounds, Estimated Position Uncertainty
        int or string: 95% vertical accuracy bounds, Vertical Estimated Position Uncertainty
    """
    tc = typecode(msg)

    if tc not in [29, 31]:
        raise RuntimeError(
            "%s: Not a target state and status message, \
                           or operation status message, expecting TC = 29 or 31"
            % msg
        )

    msgbin = common.hex2bin(msg)

    if tc == 29:
        NACp = common.bin2int(msgbin[71:75])
    elif tc == 31:
        NACp = common.bin2int(msgbin[76:80])

    try:
        EPU = uncertainty.NACp[NACp]["EPU"]
        VEPU = uncertainty.NACp[NACp]["VEPU"]
    except KeyError:
        EPU, VEPU = uncertainty.NA, uncertainty.NA

    return EPU, VEPU


def nac_v(msg):
    """Calculate NACv, Navigation Accuracy Category - Velocity

    Args:
        msg (str): 28 hexdigits string, TC = 19 or 31

    Returns:
        int or string: 95% horizontal accuracy bounds for velocity, Horizontal Figure of Merit
        int or string: 95% vertical accuracy bounds for velocity, Vertical Figure of Merit
    """
    tc = typecode(msg)

    if tc not in [19,31]:
        raise RuntimeError(
            "%s: Not an airborne velocity or surface operation status message, expecting TC = 19 or 31" % msg
        )

    msgbin = common.hex2bin(msg)
    if (tc == 19):
        NACv = common.bin2int(msgbin[42:45])
    else: #tc==31
        NACv = common.bin2int(msgbin[48:51])
    try:
        HFOMr = uncertainty.NACvC[NACv]
        VFOMr = uncertainty.NACv[NACv]["VFOMr"]
    except KeyError:
        HFOMr, VFOMr = uncertainty.NA, uncertainty.NA

    return HFOMr, VFOMr

def nac_vC(msg):
    """Calculate NACv, Navigation Accuracy Category - Velocity DO-260C

    Args:
        msg (str): 28 hexdigits string, TC = 19

    Returns:
        int or string: 95% horizontal accuracy bounds for velocity
    """
    tc = typecode(msg)

    if tc != 19:
        raise RuntimeError(
            "%s: Not an airborne velocity message, expecting TC = 19" % msg
        )

    msgbin = common.hex2bin(msg)
    NACv = common.bin2int(msgbin[42:45])

    try:
        HVE = uncertainty.NACvC[NACv]
    except KeyError:
        HVE = "<10 m/s"

    return HVE

def sil(msg, version):
    """Calculate SIL, Surveillance Integrity Level

    Args:
        msg (str): 28 hexdigits string with TC = 29, 31

    Returns:
        int or string: Probability of exceeding Horizontal Radius of Containment RCu
        int or string: Probability of exceeding Vertical Integrity Containment Region VPL
        string: SIL supplement based on per "hour" or "sample", or 'unknown'
    """
    tc = typecode(msg)

    if tc not in [29, 31]:
        raise RuntimeError(
            "%s: Not a target state and status message, \
                           or operation status message, expecting TC = 29 or 31"
            % msg
        )

    msgbin = common.hex2bin(msg)

    if tc == 29:
        SIL = common.bin2int(msgbin[76:78])
    elif tc == 31:
        SIL = common.bin2int(msgbin[82:84])

    try:
        PE_RCu = uncertainty.SIL[SIL]["PE_RCu"]
        PE_VPL = uncertainty.SIL[SIL]["PE_VPL"]
    except KeyError:
        PE_RCu, PE_VPL = uncertainty.NA, uncertainty.NA

    base = "unknown"

    if version >= 2:
        if tc == 29:
            SIL_SUP = common.bin2int(msgbin[39])
        elif tc == 31:
            SIL_SUP = common.bin2int(msgbin[86])

        if SIL_SUP == 0:
            base = "hour"
        elif SIL_SUP == 1:
            base = "sample"

    return PE_RCu, PE_VPL, base

def cas(msg):
    """Calculate CAS, Collision Avoidance Operational

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Operational or Not Operational
    """
    tc = typecode(msg)

    if ( ( tc != 31 ) or (msg[8:10] != b'F8')):
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31" %msg
        )

    msgbin = common.hex2bin(msg)

    if (int(msgbin[42]) == 1):
        return 'Operational'
    else:
        return 'Not Operational'


def es_In(msg):
    """Calculate 1090ES In

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Operational or Not Operational
    """
    tc = typecode(msg)

    if tc != 31:
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    if (int(msgbin[43]) == 1):
        return 'Capable'
    else:
        return 'Not Capable'


def transponderSide(msg):
    """Calculate transponder side

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Transponder side
    """
    tc = typecode(msg)

    if ( ( tc != 31 ) and (msg[8:10] != b'F8')):
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    side = common.bin2int(msgbin[46:48])
    if (side == 0):
        return 'Unknown'
    elif (side == 1):
        return 'Left/Single'
    elif (side == 2):
        return 'Right'
    else: # side == 3
        return 'Aux'


def txPower(msg):
    """Calculate transponder power

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        TX Power
    """
    tc = typecode(msg)

    if ( ( tc != 31 ) or (msg[8:10] != b'F8')):
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    power = common.bin2int(msgbin[48:50])
    if (power == 0):
        return 'Unknown'
    elif (power == 1):
        return '70W'
    elif (power == 2):
        return '125W'
    else: # power == 3
        return '200W'

def uat_In(msg):
    """Calculate UAT In

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Operational or Not Operational
    """
    tc = typecode(msg)

    if ( tc != 31 ):
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    if ((int(msgbin[50]) == 1) and (int(msgbin[37:40]) == 0))\
       or ((int(msgbin[47]) == 1) and (int(msgbin[37:40]) == 1)):
        return 'Capable'
    else:
        return 'Not Capable'



def rce(msg):
    """Calculate reduced capability equipment

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Reduced Capability status
    """
    tc = typecode(msg)

    if ( ( tc != 31 ) and (msg[8:10] != b'F8')):
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    rce = common.bin2int(msgbin[52:54])
    if (rce == 0):
        return 'Not RCE'
    elif (rce == 1):
        return 'TABS'
    elif (rce == 2):
        return 'RSVD'
    else: # rce == 3
        return 'Other'

def daa(msg):
    """Calculate reduced capability equipment

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Operational or Not Operational
    """
    tc = typecode(msg)

    if ( ( tc != 31 ) and (msg[8:10] != b'F8')):
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    daa = common.bin2int(msgbin[54:56])
    if (daa == 0):
        return 'No RWC'
    elif (daa == 1):
        return 'TCAS'
    elif (daa == 2):
        return 'ADSB'
    else: # daa == 3
        return 'Rsvd'


def b2(msg):
    """Calculate B2

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Operational or Not Operational
    """
    tc = typecode(msg)

    if ( ( tc != 31 ) and (msg[8:10] != b'F8')):
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    if (int(msgbin[46]) == 1):
        return 'B2'
    else:
        return 'Not B2'


def gva(msg):
    """Calculate gva, Geometric Vertical Accuracy DO-260C

    Args:
        msg (str): 28 hexdigits string, TC = 19

    Returns:
        int or string: 95% vertical accuracy bounds for position
    """
    tc = typecode(msg)

    if tc != 31 or (msg[8:10] != b'F8'):
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31" % msg
        )

    msgbin = common.hex2bin(msg)
    gva = common.bin2int(msgbin[80:82])

    try:
        return uncertainty.GVA[gva]
    except:
        return ">150 m"



def trk_hdg_true_mag(msg):
    """return if track or heading is broadcast and if its reference to true or magnetic north DO-260C

    Args:
        msg (str): 28 hexdigits string, TC = 19

    Returns:
        string Track or Heading and Magnetic or True
    """
    tc = typecode(msg)

    if tc != 31 or (msg[8:10] != b'F9'):
        raise RuntimeError(
            "%s: Not a surface operation status message, expecting TC = 31" % msg
        )

    msgbin = common.hex2bin(msg)
    hdg = common.bin2int(msgbin[84:86])

    if (hdg == 0):
        return 'True Track'
    elif (hdg == 1):
        return 'Mag. Track'
    elif (hdg == 2):
        return 'True Heading'
    else: # hdg == 3
        return 'Mag. Heading'


def ca_ra_active(msg):
    """Calculate TCAS active

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Active, Inactive
    """
    tc = typecode(msg)

    if ( tc != 31 ):
        raise RuntimeError(
            "%s: Not an operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    if (int(msgbin[58]) == 1):
        return 'Active'
    else:
        return 'Inactive'


def ident_active(msg):
    """Calculate ident active

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Active, Inactive
    """
    tc = typecode(msg)

    if ( tc != 31 ):
        raise RuntimeError(
            "%s: Not an operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    if (int(msgbin[59]) == 1):
        return 'Active'
    else:
        return 'Inactive'


def modeS_rateLimit(msg):
    """Calculate Mode-S rate kimiting status active

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Active, Inactive
    """
    tc = typecode(msg)

    if ( tc != 31 ) :
        raise RuntimeError(
            "%s: Not an operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    if (int(msgbin[60]) == 1):
        return 'Active'
    else:
        return 'Inactive'


def singleAntenna(msg):
    """Calculate Single Antenna status 

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Single, Diversity
    """
    tc = typecode(msg)

    if ( tc != 31 ):
        raise RuntimeError(
            "%s: Not an operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    if int(msgbin[61]) == 1:
        return 'Single'
    else:
        return 'Diversity'


def sda(msg):
    """return SDA DO-260C

    Args:
        msg (str): 28 hexdigits string, TC = 19

    Returns:
        string dal
    """
    tc = typecode(msg)

    if tc != 31:
        raise RuntimeError(
            "%s: Not an operation status message, expecting TC = 31" % msg
        )

    msgbin = common.hex2bin(msg)
    sda = common.bin2int(msgbin[62:64])

    if (sda == 0):
        return 'n/a'
    elif (sda == 1):
        return 'D'
    elif (sda == 2):
        return 'C'
    else: # sda == 3
        return 'B'


def cccb(msg):
    """return Collision Avoidance Coordination Capability Bits DO-260C

    Args:
        msg (str): 28 hexdigits string, TC = 19

    Returns:
        int
    """
    tc = typecode(msg)

    if tc != 31 or (msg[8:10] != b'F8'):
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31" % msg
        )

    msgbin = common.hex2bin(msg)
    
    return common.bin2int(msgbin[64:71])

def rwc(msg):
    """Indicate if DAA is issuing a RWC corrective alert

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Active, Inactive
    """
    tc = typecode(msg)

    if ( ( tc != 31 ) and (msg[8:10] != b'F8')):
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    if (int(msgbin[71])== 1):
        return 'Active'
    else:
        return 'Inactive'


def antennaOffset(msg):
    """return gps and transponder antenna offset DO-260C

    Args:
        msg (str): 28 hexdigits string, TC = 19

    Returns:
        int
    """
    tc = typecode(msg)

    if tc != 31 or (msg[8:10] != b'F9'):
        raise RuntimeError(
            "%s: Not a surface operation status message, expecting TC = 31" % msg
        )

    msgbin = common.hex2bin(msg)

    if (common.bin2int(msgbin[56:58]) == 0):
        gpsLat = common.bin2int(msgbin[67:72])
        gpsLon = common.bin2int(msgbin[65:67])
        if msgbin[64]:
            return "R {} Aft {}".format(gpsLon,(gpsLat-2)*2), None
        else:
            return "L'{} Aft {}".format(gpsLon,(gpsLat-2)*2), None
    else:
        return None, "Aft {}".format(common.bin2int(msgbin[67:72]))

def lw(msg):
    """return Length and Width Code DO-260C

    Args:
        msg (str): 28 hexdigits string, TC = 19

    Returns:
        int
    """
    tc = typecode(msg)

    if tc != 31 or (msg[8:10] != b'F9'):
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31" % msg
        )

    msgbin = common.hex2bin(msg)
    
    return common.bin2int(msgbin[52:56])


def nicBaro(msg):
    """Indicate if NicBaro is Gillham or not

    Args:
        msg (str): 28 hexdigits string with TC = 31

    Returns:
        Active, Inactive
    """
    tc = typecode(msg)

    if ( ( tc != 31 ) and (msg[8:10] != b'F8')):
        raise RuntimeError(
            "%s: Not an airborne operation status message, expecting TC = 31"
        )

    msgbin = common.hex2bin(msg)

    if (int(msgbin[84])== 1):
        return 'Not_Gillham'
    else:
        return 'Gillham'













