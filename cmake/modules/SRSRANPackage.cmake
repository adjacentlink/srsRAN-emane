#
# Copyright 2013-2022 Software Radio Systems Limited
#
# This file is part of srsRAN
#
# srsRAN is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# srsRAN is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Affero General Public License for more details.
#
# A copy of the GNU Affero General Public License can be found in
# the LICENSE file in the top-level directory of this distribution
# and at http://www.gnu.org/licenses/.
#

SET(CPACK_PACKAGE_DESCRIPTION "srsRAN EMANE")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "LTE library integrated to run on EMANE.")
SET(CPACK_PACKAGE_NAME "srsran-emane")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.3.6), libgcc1 (>= 1:4.1), libboost-dev (>= 1.35)")

SET(CPACK_PACKAGE_CONTACT "Ismael Gomez ")
SET(CPACK_PACKAGE_VENDOR "Software Radio Systems Limited")
SET(CPACK_PACKAGE_VERSION_MAJOR ${SRSRAN_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${SRSRAN_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_EMANE 1)
SET(CPACK_PACKAGE_VERSION_PATCH ${SRSRAN_VERSION_PATCH})
SET(VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_EMANE}-${CPACK_PACKAGE_VERSION_PATCH}")

########################################################################
# Setup additional defines for OS types
########################################################################
find_program(LSB_RELEASE_EXEC lsb_release)
execute_process(COMMAND ${LSB_RELEASE_EXEC} -is
  OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

execute_process(COMMAND ${LSB_RELEASE_EXEC} -rs
  OUTPUT_VARIABLE LSB_RELEASE_VERSION_SHORT
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    SET(LINUX TRUE)
ENDIF()

IF(LINUX AND EXISTS "/etc/debian_version")
    SET(DEBIAN TRUE)
ENDIF()

IF(LINUX AND EXISTS "/etc/redhat-release")
    SET(REDHAT TRUE)
ENDIF()

########################################################################
# Set generator type for recognized systems
########################################################################
IF(CPACK_GENERATOR)
    #already set
ELSEIF(APPLE)
    SET(CPACK_GENERATOR PackageMaker)
ELSEIF(WIN32)
    SET(CPACK_GENERATOR NSIS)
ELSEIF(DEBIAN)
    SET(CPACK_GENERATOR DEB)
ELSEIF(REDHAT)
    SET(CPACK_GENERATOR RPM)
ELSE()
    SET(CPACK_GENERATOR TGZ)
ENDIF()

########################################################################
# Setup CPack Debian
########################################################################
IF(LSB_RELEASE_ID_SHORT STREQUAL "Ubuntu")
  IF(LSB_RELEASE_VERSION_SHORT STREQUAL "20.04")
    SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.3.6), libgcc1 (>= 1:4.1), emane-model-lte, libfftw3-single3, libboost-program-options1.71.0, libmbedcrypto3, libconfig++9v5, libsctp1, libuhd3.15.0")
  ELSEIF(LSB_RELEASE_VERSION_SHORT STREQUAL "22.04")
    SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.3.6), libgcc1 (>= 1:4.1), emane-model-lte, libfftw3-single3, libboost-program-options1.74.0, libmbedcrypto7, libconfig++9v5, libsctp1, libuhd4.1.0")
  ELSE()
    message(FATAL_ERROR "Unsupported Ubuntu Version. Quitting.")
  ENDIF()
ENDIF()

########################################################################
# Setup CPack RPM
########################################################################
SET(CPACK_RPM_PACKAGE_REQUIRES "emane-model-lte fftw-libs-single boost-program-options mbedtls libconfig lksctp-tools uhd")

########################################################################
# Setup CPack NSIS
########################################################################
SET(CPACK_NSIS_MODIFY_PATH ON)


SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_EMANE}-${CMAKE_SYSTEM_PROCESSOR}")
INCLUDE(CPack)

