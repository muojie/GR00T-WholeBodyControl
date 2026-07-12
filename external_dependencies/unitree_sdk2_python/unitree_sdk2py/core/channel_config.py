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
