#!/bin/bash
# Substitutes versions of all packages in this repo
# by Bernd Eckstein

function showSyntax() {
    echo
    echo "This script substitutes the versions in version.py and requirements.txt"
    echo "for the projects hbp_nrp_commons, hbp_nrp_cleserver and hbp_nrp_backend"
    echo
    echo "Syntax:    ./setVersion.sh [parameter]"
    echo
    echo "Parameters:"
    echo "           --set M.m.p[.devX]]   Set version. See examples below"
    echo "           --show                Show current version"
    echo "           --help                Show this help"
    echo
    echo "Examples:  ./setVersion.sh --set 0.4.2.dev5"
    echo "           ./setVersion.sh --set 0.4.3"
    echo
}


function showVersion() {
    cat hbp_nrp_backend/hbp_nrp_backend/version.py | grep VERSION
}

function testVersion() {
    version=$1
    if [[ $version =~ ^[0-9]+\.[0-9]+\.[0-9]+(\.dev[0-9]+)?$ ]]; then
        return
    else
        echo
        echo "Version string '$version' is not in the format M.m.p[.devX]"
        echo "Examples:      '0.4.2.dev5'"
        echo "               '0.4.3'"
        echo
        exit -1
    fi
}

function setVersion() {
    version=$1
    file=$2
    echo " ... "$file
    echo "'''version string - generated by setVersion.sh'''" > $file
    echo "VERSION = '$version'" >> $file
}

function subVersion() {
    version=$1
    file=$2
    echo " ... "$file
    list="hbp-nrp-commons hbp-nrp-cleserver hbp-nrp-backend"
    for i in $list; do
        sed -i "/$i/c\\$i==$version" $file
    done
}



# make sure we are in the directory of the script
cd "$(dirname "$0")"

case "$1" in
    --help)
        showSyntax
        exit
        ;;
    --show)
        showVersion
        exit
        ;;
    --set)
        # just continue
        ;;
     *)
        showSyntax
        exit
        ;;
esac


version=$2

testVersion $version # may exit

echo
echo "Setting versions to '"$version"'" in ...

setVersion $version hbp_nrp_commons/hbp_nrp_commons/version.py
setVersion $version hbp_nrp_cleserver/hbp_nrp_cleserver/version.py
setVersion $version hbp_nrp_backend/hbp_nrp_backend/version.py

subVersion $version hbp_nrp_commons/requirements.txt
subVersion $version hbp_nrp_cleserver/requirements.txt
subVersion $version hbp_nrp_backend/requirements.txt

echo done.
echo
