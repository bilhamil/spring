# Spring supplied CMake build file

SET(OSCPACK_SRC_ROOT "${ENGINE_SRC_ROOT_DIR}/lib/oscpack")

SET(oscpackSources
		"OscOutboundPacketStream"
		"OscTypes"
		"OscPrintReceivedElements"
		"OscReceivedElements"
		"ip/IpEndpointName"
		"ip/NetworkingUtils"
		"ip/UdpSocket"
	)

	
INCLUDE_DIRECTORIES( ./ip )

ADD_LIBRARY(oscpack STATIC EXCLUDE_FROM_ALL ${oscpackSources})
IF (APPLE)
	SET_TARGET_PROPERTIES(oscpack PROPERTIES COMPILE_FLAGS -DOSC_HOST_BIG_ENDIAN)
ELSE (APPLE)
	SET_TARGET_PROPERTIES(oscpack PROPERTIES COMPILE_FLAGS -DOSC_HOST_LITTLE_ENDIAN)
ENDIF (APPLE)

