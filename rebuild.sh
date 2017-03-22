#!/bin/bash
#
#  Copyright (c) 2017 - Present European Spallation Source ERIC
#
#  The dm_setup.bash is free software: you can redistribute
#  it and/or modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation, either version 2 of the
#  License, or any newer version.
#
#  This program is distributed in the hope that it will be useful, but WITHOUT
#  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
#  more details.
#
#  You should have received a copy of the GNU General Public License along with
#  this program. If not, see https://www.gnu.org/licenses/gpl-2.0.txt
#
# Author : Jeong Han Lee
# email  : han.lee@esss.se
# Date   : 
# Version : 0.0.1

declare -gr SCRIPT="$(realpath "$0")"
declare -gr TOP="$(dirname "$SCRIPT")"
declare -g  PROJECT=""

function pushd() { builtin pushd "$@" > /dev/null; }
function popd()  { builtin popd  "$@" > /dev/null; }


eval $(cat ${TOP}/Makefile | grep -E "^(PROJECT)=")


printf "You are re-building %s ? \n" "${PROJECT}"

read -p "Do you want to continue (y/n)? " answer

case ${answer:0:1} in
    y|Y )
	printf "Yes, we are removing ...... \n";
	sudo rm -rfv ${EPICS_MODULES_PATH}/${PROJECT}

    ;;
    * )
        printf "Stop here.\n";
	exit;
    ;;
esac


printf "We will install the new version of %s\n" "${PROJECT}"

pushd ${TOP}
sudo -E make install
popd

exit
