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
