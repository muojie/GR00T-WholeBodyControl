ChannelConfigHasInterface = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>
                </Interfaces>
            </General>
            <Discovery>
                <ParticipantIndex>auto</ParticipantIndex>
                <MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
            </Discovery>
            <Tracing>
                <Verbosity>config</Verbosity>
            <OutputFile>/tmp/cdds.LOG</OutputFile>
        </Tracing>
        </Domain>
    </CycloneDDS>'''

# Select the NIC by IPv4 address instead of by name.  On Windows CycloneDDS
# matches neither the friendly adapter name ("WLAN 2") nor an IP through the
# name= attribute, so cross-machine setups have no working name= form at all;
# address= is the only one that resolves.  No <Tracing> block here on purpose:
# the hard-coded /tmp/cdds.LOG path in the templates above is not writable on
# Windows.
ChannelConfigHasAddress = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface address="$__IF_ADDR__$" priority="default" multicast="default"/>
                </Interfaces>
            </General>
            <Discovery>
                <ParticipantIndex>auto</ParticipantIndex>
                <MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
            </Discovery>
        </Domain>
    </CycloneDDS>'''

# Unicast-data variant of the address= template, selected when the
# UNITREE_DDS_PEERS environment variable lists the peer IP(s).  Made for WiFi
# links: APs forward multicast at a throttled base rate and batch it into
# beacon intervals, which crushed the Isaac lockstep to ~0.4Hz (LowState sent
# at ~90Hz arrived at ~1.2Hz) while unicast traffic on the same link ran at
# 464Hz with 2.5ms RTT.
#
# AllowMulticast=spdp - NOT false: discovery (SPDP) stays on multicast, which
# is low-rate and tolerant of AP batching AND lets an unmodified multicast
# peer (the C++ deploy binary) find us; only user data is forced onto unicast.
# A full false cut discovery both ways (lowcmd dropped to 0Hz: the peer's
# multicast SPDP announcements never reached us).  <Peers> is kept as a
# unicast discovery bootstrap to speed things up.
ChannelConfigHasAddressUnicast = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface address="$__IF_ADDR__$" priority="default" multicast="default"/>
                </Interfaces>
                <AllowMulticast>spdp</AllowMulticast>
            </General>
            <Discovery>
                <ParticipantIndex>auto</ParticipantIndex>
                <MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
                <Peers>
                    $__PEERS__$
                </Peers>
            </Discovery>
            <Internal>
                <HeartbeatInterval min="2ms" max="20ms">5ms</HeartbeatInterval>
            </Internal>
        </Domain>
    </CycloneDDS>'''

ChannelConfigLoopback = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="false"/>
                </Interfaces>
                <AllowMulticast>false</AllowMulticast>
                <EnableMulticastLoopback>false</EnableMulticastLoopback>
            </General>
            <Discovery>
                <ParticipantIndex>auto</ParticipantIndex>
                <MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
            </Discovery>
            <Tracing>
                <Verbosity>config</Verbosity>
            <OutputFile>/tmp/cdds.LOG</OutputFile>
        </Tracing>
        </Domain>
    </CycloneDDS>'''

ChannelConfigAutoDetermine = '''<?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
        <Domain Id="any">
            <General>
                <Interfaces>
                    <NetworkInterface autodetermine=\"true\" priority=\"default\" multicast=\"default\" />
                </Interfaces>
            </General>
            <Discovery>
                <ParticipantIndex>auto</ParticipantIndex>
                <MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
            </Discovery>
        </Domain>
    </CycloneDDS>'''
